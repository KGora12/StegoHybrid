// scratch/channel.cc
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/mac48-address.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/data-rate.h" // DataRate

// Extra includes for ARP cache population
#include "ns3/arp-cache.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/ipv4-interface.h"
#include "ns3/node-list.h"
#include "ns3/ipv4-header.h"
#include "ns3/object-vector.h"

#include <iomanip>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include <map>
#include <cstdint>
#include <cmath>
#include <random>
#include <queue>
#include <deque>
#include <unordered_map>
#include <cstring> // memcpy
#include <fstream> // CSV output

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("HybridTxopQam");

// ===================== GLOBAL CONFIG =====================

uint32_t SEED;
static double gSimSeconds = 300.0; // --sim=

// Hybrid mode selection.
// Random mode (default): each controller step is '1' (QAM + TXOP) with
// probability gTxopProb, otherwise '0' (QAM only). With gTxopProb = 0.2 there
// are on average four '0' steps for every '1' step.
static bool gRandomMode = true;   // --random= (0 = use the fixed pattern below)
static double gTxopProb = 0.2;    // --pOne= probability of a '1' step

// Fixed cyclic pattern, used only when --random=0
//static std::string kComboPattern = "0100000000000000000001";
static std::string kComboPattern = "00000";
//static std::string kComboPattern = "11111";

// PHY/MAC timing model for airtime (tunable to match MCS3 @ 20 MHz)
static double gAirMbps = 26.0;      // --airMbps= (MCS3 net PHY rate for payload bits)
static uint32_t gMacHdrBytes = 36;  // --macHdrB= (MAC header + FCS, approx.)
static double gPlcpUs = 120.0;      // --plcpUs= (HE preamble + SIG, us)
static double gSifsUs = 16.0;       // fixed
static double gSlotUs = 9.0;        // fixed
static double gDifsUs = 34.0;       // fixed (SIFS + 2 * slot)
static double gAvgBackoffUs = 13.5; // --bkoffUs= (~CWmin/2 * slot for VO)
static double gAckUs = 44.0;        // --ackUs= (ACK incl. preamble, rough)

// SNR/EVM for QAM BER model
static double kSnrEsDb = 26.0; // --snrDb= (Es/N0 in dB for QAM demodulation)

// QAM hidden-channel model parameters
static double gStegoQuadrantHit = 0.25; // --gateHit= P(symbol quadrant matches 2 secret bits)
static double gStegoDetectProb = 0.80;  // --gateDetect= geometric/detector success (inside/outside ellipse)
static double gHiddenBerScale = 4.0;    // --hiddenBerScale= hidden BER amplification vs. overt BER (approx.)

// In-flight limit to avoid memory blowup (back-pressure)
static uint32_t gInflightLimit = 10; // --inflight=

// NOTE: legBgRate is interpreted as the main target QAM rate
static std::string gLegacyBgRate = "5Mbps";   // --legBgRate= (e.g. 5Mbps; 0Mbps disables QAM)
static uint32_t gLegacyBgPktBytes = 1000;     // kept only for CSV metadata
static double gQamTargetBps = 5e6;            // derived from gLegacyBgRate in main()

// Variable number of background stations (ordinary traffic).
// Set with --stations and --bgRate; there is no longer a legacy OnOff flow
// on the covert station itself.
static uint32_t gNumBgSta = 0;           // --stations=K
static std::string gBgRate = "25Mbps";   // --bgRate=
static uint32_t gBgPktBytes = 2000;      // --bgPktB=
static const uint16_t kBgBasePort = 10000; // base UDP port for sinks on the AP

// CSV output
static std::string gCsvPath = "hybrid_cc_results.csv"; // --csv=

// Pacing options
static bool gPauseTxopIfQamBackpress = true; // --pauseTxopIfQamBackpress=1
static double gGuardUs = 0.0;                // --guardUs=

// --- Helpers for SNR/EVM/BER ------------------------------------------------
static inline double EvmFromSnrDb(double snrEsDb) {
  const double snrLin = std::pow(10.0, snrEsDb / 10.0);
  return 1.0 / std::sqrt(snrLin);
}
static inline double SigmaFromEvm(double evm, double Es) {
  return evm * std::sqrt(Es / 2.0);
}
static inline double Qfunc(double x) {
  return 0.5 * std::erfc(x / std::sqrt(2.0));
}
static inline double BerMqam(double snrEbLin, uint32_t M) {
  const double k = (3.0 * std::log2(M)) / ((double)M - 1.0);
  const double a = 4.0 / std::log2(M) * (1.0 - 1.0 / std::sqrt((double)M));
  return a * Qfunc(std::sqrt(k * snrEbLin));
}

// ==================== ARP CACHE POPULATION ====================
// Pre-populates the ARP cache of every node so that Wi-Fi traffic is not
// delayed by ARP resolution. Must be called after IP addresses are assigned.
// Based on a snippet by Pavel Boyko.
void PopulateARPcache() {
  Ptr<ArpCache> arp = CreateObject<ArpCache>();
  arp->SetAliveTimeout(Seconds(3600 * 24 * 365));

  // First pass: create an ARP entry for every interface address
  for (NodeList::Iterator i = NodeList::Begin(); i != NodeList::End(); ++i) {
    Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol>();
    NS_ASSERT(ip);

    ObjectVectorValue interfaces;
    ip->GetAttribute("InterfaceList", interfaces);

    for (ObjectVectorValue::Iterator j = interfaces.Begin();
         j != interfaces.End(); ++j) {
      Ptr<Ipv4Interface> ipIface = (*j).second->GetObject<Ipv4Interface>();
      NS_ASSERT(ipIface);

      Ptr<NetDevice> device = ipIface->GetDevice();
      NS_ASSERT(device);

      Mac48Address addr = Mac48Address::ConvertFrom(device->GetAddress());

      for (uint32_t k = 0; k < ipIface->GetNAddresses(); k++) {
        Ipv4Address ipAddr = ipIface->GetAddress(k).GetLocal();
        if (ipAddr == Ipv4Address::GetLoopback()) {
          continue;
        }

        ArpCache::Entry *entry = arp->Add(ipAddr);
        Ipv4Header ipv4Hdr;
        ipv4Hdr.SetDestination(ipAddr);
        Ptr<Packet> p = Create<Packet>(100);
        entry->MarkWaitReply(ArpCache::Ipv4PayloadHeaderPair(p, ipv4Hdr));
        entry->MarkAlive(addr);
      }
    }
  }

  // Second pass: attach the shared cache to every interface
  for (NodeList::Iterator i = NodeList::Begin(); i != NodeList::End(); ++i) {
    Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol>();
    NS_ASSERT(ip);

    ObjectVectorValue interfaces;
    ip->GetAttribute("InterfaceList", interfaces);

    for (ObjectVectorValue::Iterator j = interfaces.Begin();
         j != interfaces.End(); ++j) {
      Ptr<Ipv4Interface> ipIface = (*j).second->GetObject<Ipv4Interface>();
      NS_ASSERT(ipIface);
      ipIface->SetAttribute("ArpCache", PointerValue(arp));
    }
  }
}


// =========================== TXOP CONFIG ===========================
static uint32_t kAggScale = 3;           // --aggScale= bytes per unit of symbol value
static const uint32_t kAggHdrBytes = 8;  // fixed app header (8 B timestamp)
static const bool kAlternateAc = true;   // alternate VO/VI

struct AcCfg {
  uint8_t tid;
  Time txop;
  std::string edcaPath;
  uint8_t dscp;
};
static AcCfg kAcVoice{6, MicroSeconds(1504), "VO_EdcaTxopN", 0xB8}; // EF
static AcCfg kAcVideo{5, MicroSeconds(3008), "VI_EdcaTxopN", 0x88}; // AF41

static const uint16_t kBgPort = 8000; // legacy single flow (definition only, unused)
static const uint16_t kCovertPortVO = 9001;
static const uint16_t kCovertPortVI = 9002;

// =========================== QAM CONFIG ============================
static uint32_t kQamDataSize = 2000; // set with --qamPktB=
static const uint16_t kQamPort = 9009;
static const double kQamEs = 10.0;
static std::string kQamSecretMsg = "";

// ============================ HELPERS ==============================
static std::vector<uint8_t> BitsFromString(const std::string &s) {
  std::vector<uint8_t> b;
  b.reserve(8 * s.size());
  for (unsigned char c : s)
    for (int i = 7; i >= 0; --i)
      b.push_back((c >> i) & 1u);
  return b;
}
static inline void AppendBitsFromValue(std::vector<uint8_t> &out, uint32_t value, uint32_t nbits) {
  for (int i = (int)nbits - 1; i >= 0; --i)
    out.push_back((value >> i) & 1u);
}

// Airtime estimate (us) for a single 802.11 data frame carrying payloadBytes
static double AirTimeUsForPayload(uint32_t payloadBytes) {
  const double bits = 8.0 * (payloadBytes + gMacHdrBytes);
  const double txUs = (bits / (gAirMbps * 1e6)) * 1e6; // b / (b/s) -> s -> us
  return gPlcpUs + txUs + gSifsUs + gAckUs + gDifsUs + gAvgBackoffUs;
}

static std::string PseudoRandomString(size_t len, uint64_t seed,
                                      const std::string &alphabet =
                                          "abcdefghijklmnopqrstuvwxyz"
                                          "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                                          "0123456789"
                                          "_-;,':.!?/()[]{}|~@#$%^&*+=") {
  std::mt19937_64 gen(seed);
  std::uniform_int_distribution<size_t> dist(0, alphabet.size() - 1);
  std::string s;
  s.reserve(len);
  for (size_t i = 0; i < len; ++i)
    s.push_back(alphabet[dist(gen)]);
  return s;
}

// =========================== TXOP (CH#1) ===========================

struct Symbol {
  uint32_t value;
  const AcCfg *ac;
  uint32_t bitsCarried;
};

static std::vector<Symbol> PackBitsToSymbols(const std::vector<uint8_t> &bits) {
  std::vector<Symbol> out;
  size_t i = 0;
  uint32_t idx = 0;
  while (i < bits.size()) {
    bool useVi = (kAlternateAc && (idx % 2 == 1));
    uint32_t nbits = useVi ? 11u : 10u;
    uint32_t val = 0;
    for (uint32_t b = 0; b < nbits; ++b) {
      val <<= 1;
      if (i < bits.size())
        val |= (bits[i++] & 1u);
    }
    out.push_back(Symbol{val, useVi ? &kAcVideo : &kAcVoice, nbits});
    ++idx;
  }
  return out;
}

class CovertTxApp : public Application {
public:
  void Setup(Ptr<NetDevice> dev, Ptr<WifiNetDevice> wifiDev, Ipv4Address apAddr) {
    m_dev = dev;
    m_wifi = wifiDev;
    m_apAddr = apAddr;
  }
  void LoadSymbols(const std::vector<Symbol> &symbols) { m_symbols = symbols; }
  bool HasSymbols() const { return !m_symbols.empty(); }

  void SendNextSymbol() {
    if (!HasSymbols())
      return;
    m_lastAirtimeUs = 0.0;
    const Symbol &s = m_symbols[m_symIdx];
    m_symIdx = (m_symIdx + 1) % m_symbols.size();
    const AcCfg &ac = *s.ac;
    Ptr<Socket> sock = (ac.tid == kAcVoice.tid) ? m_sockVO : m_sockVI;

    // nominal TXOP for this symbol (VO = 1504 us, VI = 3008 us)
    m_lastNominalTxopUs = ac.txop.GetMicroSeconds();

    // app-level aggregation: one datagram per symbol
    const uint32_t symbolBytes = s.value * kAggScale; // may be 0
    const uint32_t wireBytes = kAggHdrBytes + symbolBytes;
    std::vector<uint8_t> wire(wireBytes, 0);

    uint64_t tsUs = (uint64_t)Simulator::Now().GetMicroSeconds(); // timestamp in the first 8 bytes
    std::memcpy(wire.data(), &tsUs, sizeof(tsUs));

    Ptr<Packet> p = Create<Packet>(wire.data(), wire.size());
    sock->Send(p);

    // actual airtime of the TXOP frame
    m_lastAirtimeUs += AirTimeUsForPayload(wireBytes);
  }

  double LastSentAirtimeUs() const { return m_lastAirtimeUs; }
  double LastNominalTxopUs() const { return m_lastNominalTxopUs; }

private:
  Ptr<NetDevice> m_dev;
  Ptr<WifiNetDevice> m_wifi;
  Ipv4Address m_apAddr;
  std::vector<Symbol> m_symbols;
  uint32_t m_symIdx = 0;
  Ptr<Socket> m_sockVO, m_sockVI;
  mutable double m_lastAirtimeUs = 0.0;
  mutable double m_lastNominalTxopUs = 0.0;

  void StartApplication() override {
    m_sockVO = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_sockVO->SetIpTos(kAcVoice.dscp);
    m_sockVO->SetPriority(kAcVoice.tid);
    m_sockVO->Connect(InetSocketAddress(m_apAddr, kCovertPortVO));

    m_sockVI = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_sockVI->SetIpTos(kAcVideo.dscp);
    m_sockVI->SetPriority(kAcVideo.tid);
    m_sockVI->Connect(InetSocketAddress(m_apAddr, kCovertPortVI));
  }
};

class CovertRxApp : public Application {
public:
  void Setup(Ptr<WifiNetDevice> wifiDev) { m_wifi = wifiDev; }
  const std::vector<uint8_t> &GetRxBits() const { return m_rxBits; }

  struct Stats {
    uint64_t bitsDecoded = 0;
    double seconds = 0.0;
    uint64_t delaySamples = 0;
    double avgDelayUs = 0.0;
    double jitterUs = 0.0;
    uint64_t voPkts = 0;
    uint64_t viPkts = 0;
  };
  Stats GetStats() const {
    Stats s;
    s.bitsDecoded = m_bitsDecoded;
    s.seconds = m_haveWindow ? (m_lastRx.GetSeconds() - m_firstRx.GetSeconds()) : 0.0;
    s.delaySamples = m_delaySamples;
    s.avgDelayUs = (m_delaySamples ? m_delaySumUs / (double)m_delaySamples : 0.0);
    s.jitterUs = m_jitterUs;
    s.voPkts = m_voPkts;
    s.viPkts = m_viPkts;
    return s;
  }

private:
  Ptr<WifiNetDevice> m_wifi;
  Ptr<Socket> m_sockVO, m_sockVI;
  std::vector<uint8_t> m_rxBits;
  bool m_haveWindow = false;
  Time m_firstRx, m_lastRx;
  uint64_t m_bitsDecoded = 0;

  // delay/jitter metrics (RFC 3550 jitter estimator)
  uint64_t m_delaySamples = 0;
  double m_delaySumUs = 0.0;
  double m_jitterUs = 0.0;
  bool m_hasPrevTransit = false;
  double m_prevTransitUs = 0.0;

  // RX packet counters per AC
  uint64_t m_voPkts = 0, m_viPkts = 0;

  void StartApplication() override {
    m_sockVO = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_sockVO->Bind(InetSocketAddress(Ipv4Address::GetAny(), kCovertPortVO));
    m_sockVO->SetRecvCallback(MakeCallback(&CovertRxApp::OnRxVO, this));

    m_sockVI = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_sockVI->Bind(InetSocketAddress(Ipv4Address::GetAny(), kCovertPortVI));
    m_sockVI->SetRecvCallback(MakeCallback(&CovertRxApp::OnRxVI, this));
  }

  void OnRxVO(Ptr<Socket> sock) {
    ++m_voPkts;
    OnRxCommon(sock, 6);
  }
  void OnRxVI(Ptr<Socket> sock) {
    ++m_viPkts;
    OnRxCommon(sock, 5);
  }

  void OnRxCommon(Ptr<Socket> sock, uint8_t tid) {
    Address from;
    Ptr<Packet> p;
    while ((p = sock->RecvFrom(from))) {
      if (!m_haveWindow) {
        m_haveWindow = true;
        m_firstRx = Simulator::Now();
      }
      m_lastRx = Simulator::Now();

      // timestamp and one-way delay
      if (p->GetSize() >= kAggHdrBytes) {
        uint8_t hdr[kAggHdrBytes];
        p->CopyData(hdr, kAggHdrBytes);
        uint64_t tsUs = 0;
        std::memcpy(&tsUs, hdr, sizeof(tsUs));
        double transitUs = (double)(Simulator::Now().GetMicroSeconds() - tsUs);
        m_delaySumUs += transitUs;
        ++m_delaySamples;
        if (m_hasPrevTransit) {
          double d = std::fabs(transitUs - m_prevTransitUs);
          m_jitterUs += (d - m_jitterUs) / 16.0;
        } else {
          m_hasPrevTransit = true;
        }
        m_prevTransitUs = transitUs;
      }

      // one packet = one symbol: decode the value from the packet size
      const uint32_t nbits = (tid == 6) ? 10u : 11u;
      const uint32_t maxVal = (1u << nbits) - 1u;
      uint32_t pktBytes = p->GetSize();
      uint32_t symBytes = (pktBytes >= kAggHdrBytes) ? (pktBytes - kAggHdrBytes) : 0u;
      uint32_t est = (uint32_t)std::llround(double(symBytes) / double(kAggScale));
      if (est > maxVal)
        est = maxVal;
      AppendBitsFromValue(m_rxBits, est, nbits);
      m_bitsDecoded += nbits;
    }
  }
};

// ========================= QAM-16 STEGO (CH#2, analytical) ===================

struct QamCtx {
  std::mt19937_64 gen{SEED};
  std::uniform_int_distribution<int> randByte{0, 255};
  std::deque<uint32_t> inflight;
  uint32_t inflightLimit = gInflightLimit;

  bool haveWindow = false;
  Time firstRx, lastRx;

  uint64_t totalPackets = 0, errorPackets = 0;
  uint64_t totalBits = 0, errorBits = 0;
  uint64_t overtBitsOk = 0;

  uint64_t hiddenBitsAttempted = 0;
  uint64_t hiddenBitsDetected = 0;
  uint64_t hiddenBitsErrors = 0;

  uint32_t nextSeq = 1;

  uint64_t delaySamples = 0;
  double delaySumUs = 0.0;
  double jitterUs = 0.0;
  bool hasPrevTransit = false;
  double prevTransitUs = 0.0;
};

class QamApp : public Application {
public:
  void Setup(Ptr<Node> txNode, Ptr<Node> rxNode, Ipv4Address rxAddr, uint16_t port) {
    m_txNode = txNode;
    m_rxNode = rxNode;
    m_rxAddr = rxAddr;
    m_port = port;
  }
  void Init() {
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    m_recv = Socket::CreateSocket(m_rxNode, tid);
    m_recv->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_port));
    m_recv->SetRecvCallback(MakeCallback(&QamApp::OnRecv, this));
    m_send = Socket::CreateSocket(m_txNode, tid);
  }

  // Returns the estimated airtime of the packet (for statistics only);
  // pacing is driven by gQamTargetBps in HybridController.
  double SendOnePacketIfPossible() {
    if (m_q.inflight.size() >= m_q.inflightLimit)
      return 0.0;

    const uint32_t headerBytes = 12; // 4 B seq + 8 B timestamp
    NS_ASSERT_MSG(kQamDataSize > headerBytes, "kQamDataSize must be > 12");

    std::vector<uint8_t> raw(kQamDataSize - headerBytes);
    for (uint32_t i = 0; i < raw.size(); ++i)
      raw[i] = (uint8_t)m_q.randByte(m_q.gen);

    uint32_t seq = m_q.nextSeq++;
    uint64_t tsUs = (uint64_t)Simulator::Now().GetMicroSeconds();

    std::vector<uint8_t> wire(headerBytes + raw.size());
    wire[0] = (seq >> 24) & 0xFF;
    wire[1] = (seq >> 16) & 0xFF;
    wire[2] = (seq >> 8) & 0xFF;
    wire[3] = seq & 0xFF;
    std::memcpy(&wire[4], &tsUs, 8);
    std::copy(raw.begin(), raw.end(), wire.begin() + headerBytes);

    Ptr<Packet> pkt = Create<Packet>(wire.data(), wire.size());
    m_send->SendTo(pkt, 0, InetSocketAddress(m_rxAddr, m_port));

    ++m_q.totalPackets;
    m_pktStore[seq] = std::move(raw);
    m_q.inflight.push_back(seq);

    return AirTimeUsForPayload(kQamDataSize);
  }

  const QamCtx &GetCtx() const { return m_q; }

private:
  Ptr<Node> m_txNode, m_rxNode;
  Ipv4Address m_rxAddr;
  uint16_t m_port{kQamPort};
  Ptr<Socket> m_send, m_recv;
  QamCtx m_q;
  std::unordered_map<uint32_t, std::vector<uint8_t>> m_pktStore;

  void StartApplication() override {
    Init();
    const uint32_t bitsPerPkt = 8u * kQamDataSize;
    const uint32_t M = 16;
    const double L = std::log2((double)M);
    const double snrEsLin = std::pow(10.0, kSnrEsDb / 10.0);
    const double snrEbLin = snrEsLin / L;
    double pb = BerMqam(snrEbLin, M);
    pb = std::clamp(pb, 1e-12, 0.49);
    m_binomOvert = std::binomial_distribution<uint32_t>(bitsPerPkt, pb);
    m_pbOvert = pb;
    m_pbHidden = std::min(0.49, m_pbOvert * gHiddenBerRatio());
  }

  double gHiddenBerRatio() const { return gHiddenBerScale; }

  void OnRecv(Ptr<Socket> sock) {
    Address from;
    Ptr<Packet> pkt = sock->RecvFrom(from);

    if (!m_q.haveWindow) {
      m_q.haveWindow = true;
      m_q.firstRx = Simulator::Now();
    }
    m_q.lastRx = Simulator::Now();

    std::vector<uint8_t> buf(pkt->GetSize());
    pkt->CopyData(buf.data(), buf.size());
    if (buf.size() < 12)
      return;

    uint32_t seq = (uint32_t(buf[0]) << 24) | (uint32_t(buf[1]) << 16) |
                   (uint32_t(buf[2]) << 8) | uint32_t(buf[3]);
    uint64_t tsUs = 0;
    std::memcpy(&tsUs, buf.data() + 4, 8);

    auto it = m_pktStore.find(seq);
    if (it == m_pktStore.end())
      return;

    if (!m_q.inflight.empty() && m_q.inflight.front() == seq)
      m_q.inflight.pop_front();

    const std::vector<uint8_t> &ref = it->second;
    const uint32_t bits = 8u * (uint32_t)ref.size();
    m_q.totalBits += bits;

    double transitUs = (double)(Simulator::Now().GetMicroSeconds() - tsUs);
    m_q.delaySumUs += transitUs;
    ++m_q.delaySamples;
    if (m_q.hasPrevTransit) {
      double d = std::fabs(transitUs - m_q.prevTransitUs);
      m_q.jitterUs += (d - m_q.jitterUs) / 16.0;
    } else {
      m_q.hasPrevTransit = true;
    }
    m_q.prevTransitUs = transitUs;

    uint32_t errBits = m_binomOvert(m_q.gen);
    m_q.errorBits += errBits;
    m_q.overtBitsOk += (bits - errBits);
    if (errBits > 0)
      ++m_q.errorPackets;

    const uint32_t symPerPkt = (uint32_t)ref.size() * 2u;
    const uint32_t hiddenCandidates =
        (uint32_t)std::llround(symPerPkt * gStegoQuadrantHit);
    const uint32_t hiddenAttempt =
        2u * (uint32_t)std::llround(hiddenCandidates * gStegoDetectProb);
    m_q.hiddenBitsAttempted += hiddenAttempt;
    if (hiddenAttempt > 0) {
      std::binomial_distribution<uint32_t> binomHidden(hiddenAttempt, m_pbHidden);
      uint32_t herr = binomHidden(m_q.gen);
      m_q.hiddenBitsErrors += herr;
      m_q.hiddenBitsDetected += (hiddenAttempt - herr);
    }

    m_pktStore.erase(it);
  }

  std::binomial_distribution<uint32_t> m_binomOvert{0, 0.0};
  double m_pbOvert = 0.0;
  double m_pbHidden = 0.0;
};

// ================= CONTROLLER (random or pattern-driven, airtime-paced) =======

class HybridController : public Application {
public:
  void Setup(Ptr<CovertTxApp> txopTx, Ptr<QamApp> qamApp) {
    m_txopTx = txopTx;
    m_qam = qamApp;
  }
  void SetPattern(const std::string &pat) { m_pattern = pat.empty() ? "0" : pat; }

  // Random mode: each step is '1' with probability pOne, otherwise '0'.
  // Uses the ns-3 RNG, so runs are reproducible for a given seed.
  void SetRandomMode(double pOne) {
    m_random = true;
    m_pOne = std::clamp(pOne, 0.0, 1.0);
    m_uv = CreateObject<UniformRandomVariable>();
  }

  void StartController() {
    Simulator::Schedule(Seconds(1.0), &HybridController::NextStep, this);
  }

  struct CStats {
    uint64_t steps = 0, ones = 0, zeros = 0, txopSends = 0;
    double sumDtUs = 0.0;
    uint64_t nZero = 0, nOne = 0;
    double sumZeroDt = 0.0, sumOneDt = 0.0;
    double sumQamUs = 0.0, sumTxopUs = 0.0;
  };

  CStats GetStats() const { return m_stats; }
  std::string GetPatternHead(uint32_t n = 64) const {
    if (m_pattern.size() <= n)
      return m_pattern;
    return m_pattern.substr(0, n);
  }

private:
  Ptr<CovertTxApp> m_txopTx;
  Ptr<QamApp> m_qam;
  std::string m_pattern{"0"};
  uint32_t m_patIdx = 0;

  bool m_random = false;
  double m_pOne = 0.0;
  Ptr<UniformRandomVariable> m_uv;

  CStats m_stats;

  void StartApplication() override {}

  // Picks the mode of the next step: drawn at random or taken from the pattern.
  char NextMode() {
    if (m_random)
      return (m_uv->GetValue(0.0, 1.0) < m_pOne) ? '1' : '0';

    const char mode = m_pattern[m_patIdx];
    m_patIdx = (m_patIdx + 1) % m_pattern.size();
    return mode;
  }

  // '0' -> 1x QAM
  // '1' -> 1x QAM + 1x TXOP symbol (single datagram)
  void NextStep() {
    if (!m_random && m_pattern.empty()) {
      Simulator::Schedule(MicroSeconds(100), &HybridController::NextStep, this);
      return;
    }

    ++m_stats.steps;
    const char mode = NextMode();

    // 1) QAM: send one packet per step if gQamTargetBps > 0.
    // The returned airtime is used for statistics; pacing follows gQamTargetBps.
    double qamAirtimeUs = 0.0;
    bool qamSent = false;
    if (gQamTargetBps > 0.0) {
      qamAirtimeUs = m_qam->SendOnePacketIfPossible(); // 0.0 if the in-flight window is full
      qamSent = (qamAirtimeUs > 0.0);
    }

    // 2) TXOP (optional)
    double txopUsActual = 0.0;
    double txopUsNominal = 0.0;
    const bool qamBackpress = (gQamTargetBps > 0.0 && !qamSent);
    bool didTxop = false;

    if (mode == '1' && m_txopTx->HasSymbols()) {
      ++m_stats.ones;
      if (!(gPauseTxopIfQamBackpress && qamBackpress)) {
        m_txopTx->SendNextSymbol();
        txopUsActual = m_txopTx->LastSentAirtimeUs();
        txopUsNominal = m_txopTx->LastNominalTxopUs();
        didTxop = true;
        ++m_stats.txopSends;
      }
    } else {
      ++m_stats.zeros;
    }

    // 3) Effective medium occupancy of TXOP: at least the nominal TXOP
    double txopUsEffective =
        didTxop ? std::max(txopUsActual, txopUsNominal) : 0.0;

    // 4) QAM pacing: ideal inter-packet gap at gQamTargetBps
    double qamIdealUs = 0.0;
    if (gQamTargetBps > 0.0) {
      // time needed to send kQamDataSize bytes at gQamTargetBps [bps]
      qamIdealUs = ((double)kQamDataSize * 8.0 / gQamTargetBps) * 1e6;
    }

    // Avoid dtUs = 0 when QAM is disabled (0Mbps) and no TXOP was sent
    double dtUs = qamIdealUs + txopUsEffective + gGuardUs;
    if (dtUs < 1.0) // minimum step of 1 us
      dtUs = 1.0;

    // Statistics: accumulate actual QAM airtime and effective TXOP time
    m_stats.sumDtUs += dtUs;
    m_stats.sumQamUs += qamAirtimeUs;
    m_stats.sumTxopUs += txopUsEffective;
    if (mode == '1') {
      ++m_stats.nOne;
      m_stats.sumOneDt += dtUs;
    } else {
      ++m_stats.nZero;
      m_stats.sumZeroDt += dtUs;
    }

    Simulator::Schedule(MicroSeconds((uint64_t)std::llround(dtUs)),
                        &HybridController::NextStep, this);
  }
};

// =============================== MAIN ========================================

int main(int argc, char **argv) {
  srand(time(NULL));
  SEED = rand();
  RngSeedManager::SetSeed(SEED);

  bool enablePcap = false;

  CommandLine cmd;
  uint32_t secretLen = 64; // --secretLen=
  uint32_t txMsgLen = 256; // --txMsgLen=
  uint64_t seedSecret =
      SEED ^ 0x9E3779B97F4A7C15ULL; // --seedSecret=
  uint64_t seedTxMsg =
      SEED ^ 0xBF58476D1CE4E5B9ULL; // --seedTxMsg=

  cmd.AddValue("qamPktB",
               "Bytes per QAM packet (incl. 12B header)", kQamDataSize);
  cmd.AddValue("secretLen", "Length of pseudorandom QAM secret message",
               secretLen);
  cmd.AddValue("txMsgLen", "Length of pseudorandom TXOP source string",
               txMsgLen);
  cmd.AddValue("seedSecret", "PRNG seed for QAM secret", seedSecret);
  cmd.AddValue("seedTxMsg", "PRNG seed for TXOP text->bits", seedTxMsg);
  cmd.AddValue("pcap", "Enable PCAP", enablePcap);

  cmd.AddValue("random", "Random hybrid mode (1) or fixed cyclic pattern (0)",
               gRandomMode);
  cmd.AddValue("pOne", "Random mode: probability of a QAM+TXOP step (default=0.2)",
               gTxopProb);
  cmd.AddValue("combo", "Cyclic pattern used when random=0 (0=QAM, 1=QAM+TXOP)",
               kComboPattern);
  cmd.AddValue("snrDb", "QAM Es/N0 in dB (analytical BER)", kSnrEsDb);
  cmd.AddValue("sim", "Simulation time in seconds", gSimSeconds);

  cmd.AddValue("airMbps", "Effective PHY payload rate (Mb/s)", gAirMbps);
  cmd.AddValue("macHdrB", "MAC hdr+FCS bytes added to payload", gMacHdrBytes);
  cmd.AddValue("plcpUs", "PLCP/preamble time [us]", gPlcpUs);
  cmd.AddValue("ackUs", "ACK frame time [us]", gAckUs);
  cmd.AddValue("bkoffUs", "Average backoff [us]", gAvgBackoffUs);

  cmd.AddValue("aggScale", "TXOP bytes scale per symbol value", kAggScale);
  cmd.AddValue("inflight", "Max in-flight QAM packets", gInflightLimit);
  cmd.AddValue("gateHit", "Hidden: quadrant hit probability",
               gStegoQuadrantHit);
  cmd.AddValue("gateDetect", "Hidden: detection (geometry) probability",
               gStegoDetectProb);
  cmd.AddValue("hiddenBerScale", "Hidden BER multiplier vs overt BER",
               gHiddenBerScale);

  cmd.AddValue("stations",
               "Number of extra background STAs generating ordinary traffic",
               gNumBgSta);
  cmd.AddValue("bgRate", "Per-station background rate (e.g., 8Mbps)", gBgRate);
  cmd.AddValue("bgPktB", "Per-station background packet size (bytes)",
               gBgPktBytes);

  cmd.AddValue("legBgRate",
               "Main QAM offered IP rate (e.g., 5Mbps, 0Mbps to disable)",
               gLegacyBgRate);
  cmd.AddValue("legBgPktB",
               "Legacy background packet size (kept only for CSV metadata)",
               gLegacyBgPktBytes);

  cmd.AddValue("csv", "CSV output filepath", gCsvPath);

  // Pacing switches
  cmd.AddValue("pauseTxopIfQamBackpress",
               "Pause TXOP if QAM couldn't send in this step (default=1)",
               gPauseTxopIfQamBackpress);
  cmd.AddValue("guardUs",
               "Guard time [us] added after (QAM+TXOP) airtime (default=0)",
               gGuardUs);

  cmd.Parse(argc, argv);

  // Convert legBgRate to the target QAM rate (0Mbps disables QAM)
  try {
    DataRate qamDr(gLegacyBgRate);
    gQamTargetBps = qamDr.GetBitRate();
  } catch (...) {
    std::cerr << "[WARN] Cannot parse legBgRate=\"" << gLegacyBgRate
              << "\", falling back to 0 bps\n";
    gQamTargetBps = 0.0;
  }

  kQamSecretMsg = PseudoRandomString(
      secretLen, seedSecret,
      "abcdefghijklmnopqrstuvwxyz"
      "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
      "0123456789_-");
  const std::string txSrc = PseudoRandomString(txMsgLen, seedTxMsg);
  std::vector<uint8_t> txBits = BitsFromString(txSrc);

  // ----------------- Nodes -----------------
  NodeContainer apNode;
  apNode.Create(1);
  NodeContainer covertSta;
  covertSta.Create(1);
  NodeContainer bgStas;
  bgStas.Create(gNumBgSta);

  // Mobility
  MobilityHelper mob;
  mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mob.Install(apNode);
  mob.Install(covertSta);
  mob.Install(bgStas);
  apNode.Get(0)->GetObject<MobilityModel>()->SetPosition(
      Vector(0.0, 0.0, 0.0));
  covertSta.Get(0)->GetObject<MobilityModel>()->SetPosition(
      Vector(5.0, 0.0, 0.0));
  for (uint32_t i = 0; i < gNumBgSta; i++) {
    bgStas.Get(i)->GetObject<MobilityModel>()->SetPosition(
        Vector(4.0 + 0.5 * i, 1.0 * (i % 3), 0.0));
  }

  // Internet stack
  InternetStackHelper internet;
  internet.Install(apNode);
  internet.Install(covertSta);
  internet.Install(bgStas);

  // Channel + PHY
  YansWifiChannelHelper chan;
  chan.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  chan.AddPropagationLoss("ns3::LogDistancePropagationLossModel");
  YansWifiPhyHelper phy;
  phy.SetChannel(chan.Create());
  phy.Set("TxPowerStart", DoubleValue(30.0));
  phy.Set("TxPowerEnd", DoubleValue(30.0));
  phy.Set("TxPowerLevels", UintegerValue(1));
  phy.Set("Antennas", UintegerValue(1));
  phy.Set("ChannelSettings",
          StringValue("{1, 20, BAND_2_4GHZ, 0}")); // channel 1, 20 MHz, 2.4 GHz

  Config::SetDefault("ns3::WifiMac::QosSupported", BooleanValue(true));
  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211ax);
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode",
                               StringValue("HeMcs3"), "ControlMode",
                               StringValue("HeMcs0"));

  WifiMacHelper mac;
  Ssid ssid = Ssid("hybrid-ssid");

  // AP
  mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
  NetDeviceContainer apDev = wifi.Install(phy, mac, apNode.Get(0));

  // Covert STA
  mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing",
              BooleanValue(false));
  NetDeviceContainer staDev = wifi.Install(phy, mac, covertSta.Get(0));

  // Background STAs
  NetDeviceContainer bgStaDevs;
  for (uint32_t i = 0; i < gNumBgSta; i++) {
    NetDeviceContainer one = wifi.Install(phy, mac, bgStas.Get(i));
    bgStaDevs.Add(one);
  }

  // PHY timing
  Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/Slot",
              TimeValue(MicroSeconds(gSlotUs)));
  Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/Sifs",
              TimeValue(MicroSeconds(gSifsUs)));

  // Addressing
  NetDeviceContainer allWifiDevices;
  allWifiDevices.Add(staDev);
  allWifiDevices.Add(apDev);
  allWifiDevices.Add(bgStaDevs);
  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.0.0.0", "255.255.255.0");
  Ipv4InterfaceContainer ifaces = ipv4.Assign(allWifiDevices);
  Ipv4Address apAddr = ifaces.GetAddress(1);
  Ipv4Address covertAddr = ifaces.GetAddress(0);

  // Populate ARP caches (required to avoid ARP stalls at simulation start)
  PopulateARPcache();

  Ptr<WifiNetDevice> txWifi = DynamicCast<WifiNetDevice>(staDev.Get(0));
  Ptr<WifiNetDevice> rxWifi = DynamicCast<WifiNetDevice>(apDev.Get(0));

  // Background contention: K stations sending ordinary UDP (BE) traffic to the AP
  for (uint32_t i = 0; i < gNumBgSta; i++) {
    uint16_t port = kBgBasePort + i;
    PacketSinkHelper sink("ns3::UdpSocketFactory",
                          InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sapps = sink.Install(apNode.Get(0));
    sapps.Start(Seconds(0.05 + 0.001 * i));

    OnOffHelper onoff("ns3::UdpSocketFactory",
                      InetSocketAddress(apAddr, port));
    onoff.SetConstantRate(DataRate(gBgRate), gBgPktBytes);
    onoff.SetAttribute("OnTime",
                       StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onoff.SetAttribute("OffTime",
                       StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    ApplicationContainer oapps = onoff.Install(bgStas.Get(i));
    oapps.Start(Seconds(0.10 + 0.002 * i));
  }

  // TXOP channel (infinite symbol queue)
  std::vector<Symbol> symbols = PackBitsToSymbols(txBits);
  Ptr<CovertTxApp> txopTx = CreateObject<CovertTxApp>();
  txopTx->Setup(staDev.Get(0), txWifi, apAddr);
  txopTx->LoadSymbols(symbols);
  covertSta.Get(0)->AddApplication(txopTx);
  txopTx->SetStartTime(Seconds(0.5));

  Ptr<CovertRxApp> txopRx = CreateObject<CovertRxApp>();
  txopRx->Setup(rxWifi);
  apNode.Get(0)->AddApplication(txopRx);
  txopRx->SetStartTime(Seconds(0.5));

  // QAM channel (analytical): AP -> covert STA (dst: covertAddr)
  Ptr<QamApp> qam = CreateObject<QamApp>();
  qam->Setup(apNode.Get(0), covertSta.Get(0), covertAddr, kQamPort);
  apNode.Get(0)->AddApplication(qam);
  qam->SetStartTime(Seconds(0.5));

  // Controller (random or pattern-driven, airtime-paced)
  Ptr<HybridController> ctrl = CreateObject<HybridController>();
  ctrl->Setup(txopTx, qam);
  if (gRandomMode)
    ctrl->SetRandomMode(gTxopProb);
  else
    ctrl->SetPattern(kComboPattern);
  covertSta.Get(0)->AddApplication(ctrl);
  ctrl->SetStartTime(Seconds(0.9));
  ctrl->StartController();

  // PCAP (optional)
  if (enablePcap) {
    YansWifiPhyHelper phyPcap;
    phyPcap.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
    NetDeviceContainer toDump;
    toDump.Add(staDev);
    toDump.Add(apDev);
    toDump.Add(bgStaDevs);
    phyPcap.EnablePcap("hybrid-cc", toDump, true);
  }

  // === FlowMonitor setup ===
  FlowMonitorHelper flowmonHelper;
  Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll();

  // Run
  Simulator::Stop(Seconds(gSimSeconds));
  Simulator::Run();

  // REPORT
  auto stats = txopRx->GetStats();
  std::vector<uint8_t> rxBits = txopRx->GetRxBits();
  const size_t txLen = txBits.size();
  const size_t rxLen = rxBits.size();
  const size_t L = std::min(txLen, rxLen);
  uint64_t errorsCommon = 0;
  for (size_t i = 0; i < L; ++i)
    if (txBits[i] != rxBits[i])
      ++errorsCommon;
  const double berCommon = (L > 0) ? double(errorsCommon) / double(L) : 0.0;

  const double secondsTxop = stats.seconds;
  const uint64_t bitsDecoded = stats.bitsDecoded;
  const double bpsTxop =
      (secondsTxop > 0.0) ? (double(bitsDecoded) / secondsTxop) : 0.0;

  const auto &qc = qam->GetCtx();
  double per =
      (qc.totalPackets ? double(qc.errorPackets) / qc.totalPackets : 0.0);
  double ber =
      (qc.totalBits ? double(qc.errorBits) / qc.totalBits : 0.0);
  double qamSeconds =
      (qc.haveWindow ? (qc.lastRx - qc.firstRx).GetSeconds() : 0.0);
  double overtPhyBps = (qc.totalBits > 0 && qamSeconds > 0.0)
                           ? (double)qc.overtBitsOk / qamSeconds
                           : 0.0;
  double grossPhyBpsTx = (qc.totalBits > 0 && qamSeconds > 0.0)
                             ? (double)qc.totalBits / qamSeconds
                             : 0.0;
  double hiddenGoodput =
      (qc.hiddenBitsDetected > 0 && qamSeconds > 0.0)
          ? (double)qc.hiddenBitsDetected / qamSeconds
          : 0.0;
  double hiddenBer =
      (qc.hiddenBitsDetected + qc.hiddenBitsErrors) > 0
          ? (double)qc.hiddenBitsErrors /
                (double)(qc.hiddenBitsDetected + qc.hiddenBitsErrors)
          : 0.0;

  const double evmReport = EvmFromSnrDb(kSnrEsDb);
  const double sigmaUsed = SigmaFromEvm(evmReport, kQamEs);

  // Delay/jitter (ms)
  double txopAvgDelayMs =
      (stats.delaySamples ? (stats.avgDelayUs / 1000.0) : 0.0);
  double txopJitterMs = stats.jitterUs / 1000.0;
  double qamAvgDelayMs =
      (qc.delaySamples ? (qc.delaySumUs / (double)qc.delaySamples) / 1000.0
                       : 0.0);
  double qamJitterMs = qc.jitterUs / 1000.0;

  // Controller stats
  auto cst = ctrl->GetStats();

  std::cout << "\n=== HYBRID REPORT (airtime-paced, analytical QAM) ===\n";
  if (gRandomMode) {
    std::cout << "Hybrid mode: random, P(1) = " << gTxopProb
              << " (0=QAM, 1=QAM+TXOP); observed 0:1 ratio = "
              << (cst.nOne ? double(cst.nZero) / double(cst.nOne) : 0.0)
              << "\n";
  } else {
    std::cout << "Hybrid mode: pattern (cyclic) \"" << kComboPattern
              << "\" (0=QAM, 1=QAM+TXOP)\n";
    std::cout << "Pattern head: " << ctrl->GetPatternHead(64) << "\n";
  }
  std::cout << "Simulation time: " << gSimSeconds
            << " s; QAM target rate: " << gLegacyBgRate
            << "; Stations(bg)=" << gNumBgSta << ", bgRate=" << gBgRate
            << ", bgPktB=" << gBgPktBytes << "\n";
  std::cout << "Pacing: guardUs=" << gGuardUs
            << ", pauseTxopIfQamBackpress="
            << (gPauseTxopIfQamBackpress ? "1" : "0") << "\n";
  std::cout << "[CTRL] steps=" << cst.steps << " ones=" << cst.ones
            << " zeros=" << cst.zeros << " txopSends=" << cst.txopSends
            << " avg_dt=" << std::fixed << std::setprecision(2)
            << (cst.steps ? cst.sumDtUs / cst.steps : 0.0) << " us"
            << " avg_qamUs="
            << (cst.steps ? cst.sumQamUs / cst.steps : 0.0) << " us"
            << " avg_txopUs="
            << (cst.steps ? cst.sumTxopUs / cst.steps : 0.0) << " us"
            << " avg_dt_ones="
            << (cst.nOne ? cst.sumOneDt / cst.nOne : 0.0) << " us"
            << " avg_dt_zeros="
            << (cst.nZero ? cst.sumZeroDt / cst.nZero : 0.0) << " us"
            << "\n";

  std::cout << "\n-- TXOP channel --\n";
  std::cout << "Decoded bits: " << bitsDecoded << "\n";
  std::cout << std::fixed << std::setprecision(6)
            << "Measurement window: " << secondsTxop << " s\n";
  std::cout << std::setprecision(2)
            << "Goodput: " << bpsTxop << " bps (" << bpsTxop / 1000.0
            << " kbps, " << bpsTxop / 1e6 << " Mbps)\n";
  std::cout << std::setprecision(4)
            << "BER_common: " << (berCommon * 100.0) << " %\n";
  std::cout << std::setprecision(2)
            << "Avg one-way delay: " << txopAvgDelayMs
            << " ms, jitter: " << txopJitterMs << " ms"
            << " [VO=" << stats.voPkts << ", VI=" << stats.viPkts
            << ", total=" << (stats.voPkts + stats.viPkts) << "]\n";

  std::cout << "\nQAM noise model: SNR_Es = " << std::setprecision(2)
            << kSnrEsDb << " dB, EVM = " << std::setprecision(4)
            << evmReport << " (" << std::setprecision(2)
            << evmReport * 100.0 << " %)"
            << ", sigma = " << std::setprecision(4) << sigmaUsed << "\n";

  std::cout << "\n-- QAM-16 overt (analytical BER) --\n";
  std::cout << "Packets: " << qc.totalPackets
            << ", ErrPkts: " << qc.errorPackets
            << " -> PER ~= " << std::setprecision(6) << per << "\n";
  std::cout << "Bits: " << qc.totalBits
            << ", ErrBits: " << qc.errorBits
            << " -> BER ~= " << std::setprecision(6) << ber << "\n";
  std::cout << std::setprecision(2)
            << "Overt PHY goodput: " << overtPhyBps << " bps ("
            << overtPhyBps / 1000.0 << " kbps, " << overtPhyBps / 1e6
            << " Mbps)\n"
            << "Gross PHY rate TX: " << grossPhyBpsTx << " bps ("
            << grossPhyBpsTx / 1000.0 << " kbps, "
            << grossPhyBpsTx / 1e6 << " Mbps)\n";

  std::cout << "\n-- QAM-16 hidden (model) --\n";
  std::cout << "Hidden bits attempted: " << qc.hiddenBitsAttempted
            << ", detected: " << qc.hiddenBitsDetected
            << ", errors: " << qc.hiddenBitsErrors
            << " -> BER_hidden ~= " << std::setprecision(6)
            << hiddenBer << "\n";
  std::cout << std::setprecision(2)
            << "Hidden goodput: " << hiddenGoodput << " bps ("
            << hiddenGoodput / 1000.0 << " kbps, "
            << hiddenGoodput / 1e6 << " Mbps)\n";

  std::cout << "\n-- QAM delay/jitter --\n";
  std::cout << std::setprecision(2)
            << "Avg one-way delay: " << qamAvgDelayMs
            << " ms, jitter: " << qamJitterMs << " ms"
            << " [" << qc.delaySamples << " pkts]\n";

  std::cout << "\n-- Airtime model (tunable) --\n";
  std::cout << "airMbps=" << gAirMbps << ", plcpUs=" << gPlcpUs
            << ", ackUs=" << gAckUs << ", difs=" << gDifsUs
            << ", bkoff=" << gAvgBackoffUs
            << ", macHdrB=" << gMacHdrBytes << "\n";

  // ======================= FLOWMONITOR: IP-level metrics ======================
  monitor->CheckForLostPackets();
  double offeredIpBps = 0.0;
  double rxIpBps = 0.0;
  double ipLoss = 0.0;
  double ipDelayMs = 0.0;
  double ipJitterMs = 0.0;

  Ptr<Ipv4FlowClassifier> classifier =
      DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
  auto flowStats = monitor->GetFlowStats();

  for (const auto &kv : flowStats) {
    const FlowMonitor::FlowStats &st = kv.second;
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(kv.first);
    // QAM flow: AP -> covert STA (UDP, dst port = kQamPort)
    if (t.protocol != 17)
      continue; // UDP only
    if (!(t.sourceAddress == apAddr && t.destinationAddress == covertAddr))
      continue;
    if (t.destinationPort != kQamPort)
      continue;

    double win = gSimSeconds; // whole simulation duration

    offeredIpBps = (win > 0.0) ? (st.txBytes * 8.0) / win : 0.0;
    rxIpBps = (win > 0.0) ? (st.rxBytes * 8.0) / win : 0.0;

    ipLoss =
        (st.txPackets > 0) ? double(st.lostPackets) / double(st.txPackets)
                           : 0.0;
    ipDelayMs = (st.rxPackets > 0)
                    ? (st.delaySum.GetSeconds() / st.rxPackets) * 1000.0
                    : 0.0;
    ipJitterMs =
        (st.rxPackets > 1)
            ? (st.jitterSum.GetSeconds() / (st.rxPackets - 1)) * 1000.0
            : 0.0;
  }

  std::cout << std::setprecision(2)
            << "\n[FlowMonitor QAM] offered_ip=" << offeredIpBps / 1e6
            << " Mb/s, "
            << "rx_ip=" << rxIpBps / 1e6 << " Mb/s, "
            << "loss=" << (ipLoss * 100.0) << " %, "
            << "delay=" << ipDelayMs << " ms, "
            << "jitter=" << ipJitterMs << " ms\n";

  // ============================ CSV APPEND =============================
  // Columns:
  // stations_total,snr_db,frame_bytes,
  // bg_rate,leg_bg_rate,leg_bg_pktB,
  // txop_goodput_bps,txop_delay_ms,txop_jitter_ms,
  // qam_per,qam_ber,qam_goodput_bps,
  // hidden_ber,hidden_goodput_bps,
  // qam_delay_ms,qam_jitter_ms,
  // offered_ip_bps,rx_ip_bps,ip_loss,ip_delay_ms,ip_jitter_ms
  const uint32_t stationsTotal =
      gNumBgSta + 1; // background STAs + covert STA

  // Full MPDU: payload (kQamDataSize) + MAC header/FCS (gMacHdrBytes)
  const uint32_t frameBytes = kQamDataSize + gMacHdrBytes;


  bool csvExists = false;
  {
    std::ifstream fi(gCsvPath.c_str(), std::ios::in);
    csvExists = fi.good();
  }

  {
    std::ofstream fo(gCsvPath.c_str(),
                     std::ios::out | std::ios::app);
    if (!fo) {
      std::cerr << "[CSV] ERROR: cannot open file for append: "
                << gCsvPath << "\n";
    } else {
      if (!csvExists) {
        fo << "stations_total,"
              "snr_db,"
              "frame_bytes,"
              "bg_rate,"
              "leg_bg_rate,"
              "leg_bg_pktB,"
              "txop_goodput_bps,"
              "txop_delay_ms,"
              "txop_jitter_ms,"
              "qam_per,"
              "qam_ber,"
              "qam_goodput_bps,"
              "hidden_ber,"
              "hidden_goodput_bps,"
              "qam_delay_ms,"
              "qam_jitter_ms,"
              "offered_ip_bps,"
              "rx_ip_bps,"
              "ip_loss,"
              "ip_delay_ms,"
              "ip_jitter_ms"
           << "\n";
      }

      fo << std::setprecision(10) << std::fixed << stationsTotal << ","
         << kSnrEsDb << "," << frameBytes << "," << gBgRate << ","
         << gLegacyBgRate << "," << gLegacyBgPktBytes << "," << bpsTxop
         << "," << txopAvgDelayMs << "," << txopJitterMs << "," << per
         << "," << ber << "," << overtPhyBps << "," << hiddenBer << ","
         << hiddenGoodput << "," << qamAvgDelayMs << "," << qamJitterMs
         << "," << offeredIpBps << "," << rxIpBps << "," << ipLoss << ","
         << ipDelayMs << "," << ipJitterMs << "\n";
    }
  }
  // ========================== END CSV APPEND ===========================

  Simulator::Destroy();
  return 0;
}