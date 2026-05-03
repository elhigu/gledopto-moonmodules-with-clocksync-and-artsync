#pragma once

#include "wled.h"
#include "AsyncUDP.h"
#include <sys/time.h>
#include <map>

// ---------------------------------------------------------------------------
// ClockSync usermod (beacon election + PTP-style bidirectional sync)
//
// Beacon plane (broadcast, every ~20 s, magic "WCSB"):
//   * Carries gettimeofday() + MAC + status
//   * Drives best-master-clock election (lowest MAC of any non-stale peer)
//   * Populates peer table for monitoring (Info JSON / serial)
//
// PTP plane (unicast slave→master, every ~2 s, magic "WCPT"):
//   * Slave sends Sync Request with its T1 (send time)
//   * Master replies with T1 (echoed), T2 (its receive time), T3 (its send time)
//   * Slave captures T4 (receive time) and computes
//        offset    = ((T2 − T1) + (T3 − T4)) / 2
//        roundtrip = (T4 − T1) − (T3 − T2)
//   * Symmetric callback latency cancels in offset; only true clock skew remains
//   * Slave applies offset via settimeofday (>slew_threshold_ms) or adjtime
//
// PTP supplants beacon-based slewing — running both creates oscillation as
// they have different bias structures. Beacons stay for election + monitoring.
// If PTP responses stop arriving (master changed, network blip), slewing
// pauses and the system flags "ptp stale" until exchange resumes.
//
// Settings (Usermod Settings page → ClockSync):
//   enabled                  master switch. Default: false.
//   master                   OPTIONAL manual override; empty = auto-elect.
//   beacon_interval_ms       beacon broadcast period. Default 20000.
//   ptp_interval_ms          PTP request period (slaves only). Default 2000.
//   slew_threshold_ms        |offset| above this triggers settimeofday() jump,
//                            below uses adjtime() slew. Default 5.
//   serial_debug             UI-toggleable serial status print.
//   serial_debug_interval_ms cadence for the above.
//
// Cross-node visibility: /json/info → "u" object → ClockSync entries.
// ---------------------------------------------------------------------------

#define CLOCKSYNC_BEACON_PORT     21337
#define CLOCKSYNC_BEACON_MAGIC    0x42534357UL  // "WCSB" LE
#define CLOCKSYNC_BEACON_VERSION  2
#define CLOCKSYNC_PTP_MAGIC       0x54504357UL  // "WCPT" LE
#define CLOCKSYNC_PTP_VERSION     1

class ClockSyncUsermod : public Usermod {
  private:
    bool     enabled           = false;
    char     masterIp[16]      = "";
    uint32_t beaconIntervalMs  = 20000;
    uint32_t ptpIntervalMs     = 2000;
    uint32_t slewThresholdMs   = 5;
    bool     serialDebug       = false;
    uint32_t serialDebugIntervalMs = 5000;
    bool     initDone          = false;

    uint32_t lastBeaconSentMs  = 0;
    uint32_t lastPtpSentMs     = 0;
    uint32_t lastPtpResponseMs = 0;        // millis() at last good response
    int64_t  lastPtpOffsetUs   = 0;        // signed: positive = slave behind master
    int64_t  lastPtpRoundtripUs = 0;

    AsyncUDP udp;
    bool udpStarted = false;

    uint8_t   ownMac[6]            = {0};
    uint8_t   electedMac[6]        = {0};
    IPAddress electedIp            = IPAddress(0,0,0,0);
    bool      electedSelfIsMaster  = true;

    struct PeerInfo {
      uint32_t lastSeenMs;
      uint64_t now_us;
      int64_t  delta_us;
      uint8_t  sync_status;
      uint8_t  sync_source;
      uint32_t sync_age_ms;
      uint8_t  mac[6];
    };
    std::map<uint32_t, PeerInfo> peers;
    static constexpr uint32_t PEER_PRUNE_AGE_MS = 60000;

    #pragma pack(push, 1)
    struct BeaconPacket {
      uint32_t magic;
      uint8_t  version;
      uint8_t  sync_status;
      uint8_t  sync_source;
      uint8_t  reserved0;
      uint64_t now_us;
      uint32_t sync_age_ms;
      uint8_t  mac[6];
      uint8_t  reserved1[2];
    };
    struct PtpPacket {
      uint32_t magic;
      uint8_t  version;
      uint8_t  type;          // 1 = Sync Request, 2 = Sync Response
      uint8_t  reserved[2];
      uint64_t t1;            // slave send time (echoed in response)
      uint64_t t2;            // master receive time (response only)
      uint64_t t3;            // master send time    (response only)
    };
    #pragma pack(pop)
    static_assert(sizeof(BeaconPacket) == 28, "Beacon packet size mismatch");
    static_assert(sizeof(PtpPacket) == 32,    "PTP packet size mismatch");

    static const char* statusName(uint8_t s) {
      switch (s) {
        case 1: return "no peers";
        case 2: return "slewing";
        case 3: return "synced";
        case 4: return "master";
        case 5: return "ptp stale";
        default: return "disabled";
      }
    }

    void captureOwnMac() { WiFi.macAddress(ownMac); }

    static uint64_t nowUs() {
      struct timeval tv;
      gettimeofday(&tv, nullptr);
      return (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
    }

    void applyOffset(int64_t offsetUs) {
      int64_t abs_us = offsetUs < 0 ? -offsetUs : offsetUs;
      if ((uint64_t)abs_us > (uint64_t)slewThresholdMs * 1000ULL) {
        // Big jump: set to (now + offset)
        uint64_t target = nowUs() + (uint64_t)offsetUs;
        struct timeval tv;
        tv.tv_sec  = (time_t)(target / 1000000ULL);
        tv.tv_usec = (suseconds_t)(target % 1000000ULL);
        settimeofday(&tv, nullptr);
      } else {
        struct timeval d;
        d.tv_sec  = (time_t)(offsetUs / 1000000);
        d.tv_usec = (suseconds_t)(offsetUs % 1000000);
        adjtime(&d, nullptr);
      }
    }

    void runElection() {
      uint8_t bestMac[6];
      memcpy(bestMac, ownMac, 6);
      IPAddress bestIp;
      bool selfIsMaster = true;

      uint32_t now = millis();
      uint32_t staleMs = 3 * beaconIntervalMs;
      for (auto& kv : peers) {
        if ((now - kv.second.lastSeenMs) > staleMs) continue;
        if (memcmp(kv.second.mac, bestMac, 6) < 0) {
          memcpy(bestMac, kv.second.mac, 6);
          bestIp = IPAddress(kv.first);
          selfIsMaster = false;
        }
      }
      memcpy(electedMac, bestMac, 6);
      electedIp = bestIp;
      electedSelfIsMaster = selfIsMaster;
    }

    bool currentMasterIp(IPAddress& out) const {
      if (masterIp[0] != 0) {
        IPAddress manual;
        if (manual.fromString(masterIp)) { out = manual; return true; }
      }
      if (electedSelfIsMaster) return false;
      out = electedIp;
      return true;
    }

    void updatePeerFromBeacon(IPAddress remote, const BeaconPacket& b, uint64_t our_now) {
      PeerInfo info;
      info.lastSeenMs  = millis();
      info.now_us      = b.now_us;
      info.delta_us    = (int64_t)b.now_us - (int64_t)our_now;
      info.sync_status = b.sync_status;
      info.sync_source = b.sync_source;
      info.sync_age_ms = b.sync_age_ms;
      memcpy(info.mac, b.mac, 6);
      peers[(uint32_t)remote] = info;
    }

    void handleBeacon(AsyncUDPPacket& packet, uint64_t our_now) {
      if (packet.length() != sizeof(BeaconPacket)) return;
      const BeaconPacket* b = reinterpret_cast<const BeaconPacket*>(packet.data());
      if (b->version != CLOCKSYNC_BEACON_VERSION) return;
      updatePeerFromBeacon(packet.remoteIP(), *b, our_now);
      // Note: beacon-based slewing intentionally removed; PTP handles it.
    }

    void handlePtp(AsyncUDPPacket& packet, uint64_t t_recv) {
      if (packet.length() != sizeof(PtpPacket)) return;
      const PtpPacket* p = reinterpret_cast<const PtpPacket*>(packet.data());
      if (p->version != CLOCKSYNC_PTP_VERSION) return;

      if (p->type == 1) {
        // Request — capture T2, send response with T2 + T3 immediately.
        PtpPacket resp = {};
        resp.magic   = CLOCKSYNC_PTP_MAGIC;
        resp.version = CLOCKSYNC_PTP_VERSION;
        resp.type    = 2;
        resp.t1      = p->t1;        // echo
        resp.t2      = t_recv;       // captured at callback entry
        resp.t3      = nowUs();      // just before send
        udp.writeTo(reinterpret_cast<uint8_t*>(&resp), sizeof(resp),
                    packet.remoteIP(), packet.remotePort());
        return;
      }

      if (p->type == 2) {
        // Response — accept only from configured/elected master.
        IPAddress masterAddr;
        if (!currentMasterIp(masterAddr) || packet.remoteIP() != masterAddr) return;

        int64_t T1 = (int64_t)p->t1;
        int64_t T2 = (int64_t)p->t2;
        int64_t T3 = (int64_t)p->t3;
        int64_t T4 = (int64_t)t_recv;

        int64_t offset    = ((T2 - T1) + (T3 - T4)) / 2;
        int64_t roundtrip = (T4 - T1) - (T3 - T2);

        // Sanity: discard wildly bad measurements (e.g., RTT < 0 from timestamp glitches).
        if (roundtrip < 0 || roundtrip > 1000000) return;

        lastPtpOffsetUs    = offset;
        lastPtpRoundtripUs = roundtrip;
        lastPtpResponseMs  = millis();
        applyOffset(offset);
        return;
      }
    }

    void sendBeacon() {
      if (!udpStarted) return;
      uint64_t now = nowUs();

      BeaconPacket b{};
      b.magic       = CLOCKSYNC_BEACON_MAGIC;
      b.version     = CLOCKSYNC_BEACON_VERSION;
      b.sync_status = computeSyncStatus();
      b.sync_source = (computeSyncStatus() == 3 || computeSyncStatus() == 2) ? 1 : 0;
      b.now_us      = now;
      b.sync_age_ms = (lastPtpResponseMs > 0)
                      ? (uint32_t)(millis() - lastPtpResponseMs)
                      : 0xFFFFFFFFUL;
      memcpy(b.mac, ownMac, 6);

      IPAddress bcast(255, 255, 255, 255);
      udp.writeTo(reinterpret_cast<uint8_t*>(&b), sizeof(b), bcast, CLOCKSYNC_BEACON_PORT);
    }

    void sendPtpRequest() {
      if (!udpStarted) return;
      IPAddress masterAddr;
      if (!currentMasterIp(masterAddr)) return;          // I'm master or no peer yet

      PtpPacket req = {};
      req.magic   = CLOCKSYNC_PTP_MAGIC;
      req.version = CLOCKSYNC_PTP_VERSION;
      req.type    = 1;
      req.t1      = nowUs();        // captured immediately before send

      udp.writeTo(reinterpret_cast<uint8_t*>(&req), sizeof(req),
                  masterAddr, CLOCKSYNC_BEACON_PORT);
    }

    void startUdp() {
      if (udpStarted) return;
      if (!udp.listen(CLOCKSYNC_BEACON_PORT)) return;
      udp.onPacket([this](AsyncUDPPacket packet) {
        // Capture local timestamp ASAP — minimizes callback-latency bias.
        uint64_t t_recv = nowUs();

        IPAddress remote = packet.remoteIP();
        if (remote == WiFi.localIP()) return;
#ifdef WLED_USE_ETHERNET
        if (remote == ETH.localIP()) return;
#endif

        if (packet.length() < 4) return;
        uint32_t magic = *reinterpret_cast<const uint32_t*>(packet.data());
        if (magic == CLOCKSYNC_BEACON_MAGIC) handleBeacon(packet, t_recv);
        else if (magic == CLOCKSYNC_PTP_MAGIC) handlePtp(packet, t_recv);
      });
      udpStarted = true;
    }

    void stopUdp() {
      if (!udpStarted) return;
      udp.close();
      udpStarted = false;
      peers.clear();
      lastPtpResponseMs = 0;
      lastPtpOffsetUs = 0;
      lastPtpRoundtripUs = 0;
    }

    uint8_t computeSyncStatus() const {
      if (!enabled) return 0;
      if (electedSelfIsMaster && masterIp[0] == 0) return 4;     // I am master
      if (lastPtpResponseMs == 0) return 1;                       // no master yet
      uint32_t age = millis() - lastPtpResponseMs;
      if (age > 3 * ptpIntervalMs) return 5;                      // stale
      int64_t abs_us = lastPtpOffsetUs < 0 ? -lastPtpOffsetUs : lastPtpOffsetUs;
      bool tight = (uint64_t)abs_us < 2000ULL;                    // within 2 ms
      return tight ? 3 : 2;
    }

    void prunePeers() {
      uint32_t now = millis();
      for (auto it = peers.begin(); it != peers.end();) {
        if ((now - it->second.lastSeenMs) > PEER_PRUNE_AGE_MS) {
          it = peers.erase(it);
        } else {
          ++it;
        }
      }
    }

  public:
    void setup() override {
      captureOwnMac();
      if (enabled) startUdp();
      initDone = true;
    }

    void connected() override {
      captureOwnMac();
      if (enabled && !udpStarted) startUdp();
    }

    void loop() override {
      if (!enabled) return;
      uint32_t now = millis();
      runElection();

      if (udpStarted && (now - lastBeaconSentMs) >= beaconIntervalMs) {
        lastBeaconSentMs = now;
        sendBeacon();
      }

      // Slaves only — master is whoever has lowest MAC, and they don't sync to anyone.
      bool isSlave = !electedSelfIsMaster || (masterIp[0] != 0);
      if (udpStarted && isSlave && (now - lastPtpSentMs) >= ptpIntervalMs) {
        lastPtpSentMs = now;
        sendPtpRequest();
      }

      prunePeers();

      if (serialDebug) {
        static uint32_t lastSerial = 0;
        uint32_t period = serialDebugIntervalMs < 500 ? 500 : serialDebugIntervalMs;
        if (now - lastSerial >= period) {
          lastSerial = now;
          Serial.printf("[ClockSync] state=%s peers=%u",
                        statusName(computeSyncStatus()),
                        (unsigned)peers.size());
          if (electedSelfIsMaster && masterIp[0] == 0) {
            Serial.print(" master=self");
          } else if (electedIp != IPAddress(0,0,0,0)) {
            Serial.printf(" master=%d.%d.%d.%d",
                          electedIp[0], electedIp[1], electedIp[2], electedIp[3]);
          }
          if (lastPtpResponseMs > 0) {
            Serial.printf(" offset=%+.3fms rtt=%.3fms last=%us",
                          lastPtpOffsetUs / 1000.0,
                          lastPtpRoundtripUs / 1000.0,
                          (unsigned)((millis() - lastPtpResponseMs) / 1000));
          }
          for (auto& kv : peers) {
            IPAddress p(kv.first);
            Serial.printf(" | %d.%d.%d.%d:%+.3fms(%s)",
                          p[0], p[1], p[2], p[3],
                          kv.second.delta_us / 1000.0,
                          statusName(kv.second.sync_status));
          }
          Serial.println();
        }
      }
    }

    void addToConfig(JsonObject& root) override {
      JsonObject top = root.createNestedObject(F("ClockSync"));
      top[F("enabled")]                  = enabled;
      top[F("master")]                   = masterIp;
      top[F("beacon_interval_ms")]       = beaconIntervalMs;
      top[F("ptp_interval_ms")]          = ptpIntervalMs;
      top[F("slew_threshold_ms")]        = slewThresholdMs;
      top[F("serial_debug")]             = serialDebug;
      top[F("serial_debug_interval_ms")] = serialDebugIntervalMs;
    }

    bool readFromConfig(JsonObject& root) override {
      JsonObject top = root[F("ClockSync")];
      if (top.isNull()) return false;

      bool prevEnabled = enabled;
      enabled = top[F("enabled")] | enabled;
      const char* m = top[F("master")];
      if (m) strlcpy(masterIp, m, sizeof(masterIp));
      beaconIntervalMs       = top[F("beacon_interval_ms")]       | beaconIntervalMs;
      ptpIntervalMs          = top[F("ptp_interval_ms")]          | ptpIntervalMs;
      slewThresholdMs        = top[F("slew_threshold_ms")]        | slewThresholdMs;
      serialDebug            = top[F("serial_debug")]             | serialDebug;
      serialDebugIntervalMs  = top[F("serial_debug_interval_ms")] | serialDebugIntervalMs;
      if (beaconIntervalMs       < 1000) beaconIntervalMs       = 1000;
      if (ptpIntervalMs          < 200)  ptpIntervalMs          = 200;
      if (slewThresholdMs        < 1)    slewThresholdMs        = 1;
      if (serialDebugIntervalMs  < 500)  serialDebugIntervalMs  = 500;

      if (initDone && prevEnabled != enabled) {
        if (enabled) startUdp();
        else         stopUdp();
      }
      return true;
    }

    void addToJsonInfo(JsonObject& root) override {
      JsonObject user = root[F("u")];
      if (user.isNull()) user = root.createNestedObject(F("u"));

      uint64_t now_us = nowUs();
      char nowBuf[24];
      uint32_t hi = (uint32_t)(now_us / 1000000000ULL);
      uint32_t lo = (uint32_t)(now_us % 1000000000ULL);
      if (hi > 0) snprintf(nowBuf, sizeof(nowBuf), "%u%09u", (unsigned)hi, (unsigned)lo);
      else        snprintf(nowBuf, sizeof(nowBuf), "%u",     (unsigned)lo);

      JsonArray a;
      a = user.createNestedArray(F("ClockSync now"));
      a.add(nowBuf); a.add(F("us"));

      a = user.createNestedArray(F("ClockSync state"));
      a.add(statusName(computeSyncStatus())); a.add(F(""));

      if (enabled) {
        IPAddress masterAddr;
        bool hasMaster = currentMasterIp(masterAddr);
        a = user.createNestedArray(F("ClockSync source"));
        if (masterIp[0] != 0)         a.add(masterIp);
        else if (electedSelfIsMaster) a.add("self (elected)");
        else if (hasMaster) {
          char ipbuf[28];
          snprintf(ipbuf, sizeof(ipbuf), "%d.%d.%d.%d (elected)",
                   masterAddr[0], masterAddr[1], masterAddr[2], masterAddr[3]);
          a.add(ipbuf);
        } else                        a.add("(no peers yet)");
        a.add(F(""));

        if (hasMaster && lastPtpResponseMs > 0) {
          a = user.createNestedArray(F("ClockSync offset"));
          a.add(lastPtpOffsetUs / 1000.0); a.add(F("ms"));

          a = user.createNestedArray(F("ClockSync RTT"));
          a.add(lastPtpRoundtripUs / 1000.0); a.add(F("ms"));

          a = user.createNestedArray(F("ClockSync last PTP"));
          a.add((millis() - lastPtpResponseMs) / 1000.0); a.add(F("s ago"));
        }

        char macbuf[20];
        snprintf(macbuf, sizeof(macbuf), "%02X:%02X:%02X:%02X:%02X:%02X",
                 ownMac[0], ownMac[1], ownMac[2], ownMac[3], ownMac[4], ownMac[5]);
        a = user.createNestedArray(F("ClockSync own MAC"));
        a.add(macbuf); a.add(F(""));

        for (auto& kv : peers) {
          IPAddress peerIp = IPAddress(kv.first);
          char key[40];
          snprintf(key, sizeof(key), "ClockSync peer %d.%d.%d.%d",
                   peerIp[0], peerIp[1], peerIp[2], peerIp[3]);
          char val[96];
          double delta_ms = kv.second.delta_us / 1000.0;
          uint32_t age_s = (millis() - kv.second.lastSeenMs) / 1000;
          snprintf(val, sizeof(val),
                   "%+.3fms (%s, last %us, MAC %02X:%02X:%02X:%02X:%02X:%02X)",
                   delta_ms,
                   statusName(kv.second.sync_status),
                   (unsigned)age_s,
                   kv.second.mac[0], kv.second.mac[1], kv.second.mac[2],
                   kv.second.mac[3], kv.second.mac[4], kv.second.mac[5]);
          a = user.createNestedArray(key);
          a.add(val); a.add(F(""));
        }

        a = user.createNestedArray(F("ClockSync peers"));
        a.add((int)peers.size()); a.add(F(""));
      }
    }

    uint16_t getId() override { return USERMOD_ID_CLOCKSYNC; }
};
