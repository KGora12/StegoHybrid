// QAM-16 steganography over 802.11ax (HE-MCS3, 20 MHz) with EVM-based noise derived from SNR
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"

#include <vector>
#include <queue>
#include <random>
#include <iostream>
#include <fstream>
#include <cfloat>
#include <cmath>
#include <unordered_map>
#include <deque>
#include <cstdint>
#include <cstring> // memcpy
#include <sstream> // std::ostringstream


using namespace ns3;
NS_LOG_COMPONENT_DEFINE("Qam16Stego");

// ----------------- Command-line parameters / globals -----------------
static double   gAirMbps       = 26.0;   // effective PHY rate for payload bits (MCS3 @ 20 MHz)
static uint32_t gMacHdrBytes   = 36;     // MAC header + FCS (approx.)
static double   gPlcpUs        = 120.0;  // HE preamble + SIG fields
static double   gSifsUs        = 16.0;   // SIFS
static double   gSlotUs        = 9.0;    // slot time
static double   gDifsUs        = 34.0;   // DIFS (SIFS + 2 * slot)
static double   gAvgBackoffUs  = 13.5;   // ~CWmin/2 * slot for AC_VO
static double   gAckUs         = 44.0;   // ACK including preamble
static const uint32_t SEED     = 12345;
static double   gSimSeconds    = 300.0;  // simulation time: 300 s
static uint32_t gNumBgSta      = 0;

static uint32_t kDataSize      = 2000;   // total UDP payload in bytes
static uint32_t kUdpPort       = 9;
static double   gSnrDb         = 24.0;   // set with --snrDb=XX


// --- constellation logging: first packet only ---
static bool gLogFirstPacketTx = true;   // TX: first transmitted packet
static bool gLogFirstPacketRx = true;   // RX: first received packet (seq = 0)


// --- ellipse thresholds (e) ---
static double gEllipseE1 = 1.0 / 300.0;  // smallest ellipse, used by the decoder
static double gEllipseE2 = 1.0 / 70.0;   // medium ellipse, inner bound of the embedding ring
static double gEllipseE3 = 1.0 / 40.0;   // largest ellipse, outer bound of the embedding ring



// --- fixed packet send interval ---
static const double kSendIntervalSec = 0.1; 

// ------------- Steganography state and metrics -------------
std::ofstream csvFile;


std::queue<bool> secretBitQueue;                // secret bit source, refilled with random bits on demand
std::vector<bool> secretMessageSent;
std::vector<bool> secretMessageReceived;

uint64_t totalPackets = 0, errorPackets = 0, totalBits = 0, errorBits = 0;
uint64_t secretBitsSent = 0, secretBitsDetected = 0;

// PRNG
std::mt19937 rng(SEED);

// ====== In-flight packet buffer (seq -> TX record) ======
struct TxRecord {
  std::vector<uint8_t> data; // raw user payload (without the UDP header)
  std::vector<std::pair<float,float>> symbols;
};
static uint32_t gSeq = 0;
static std::unordered_map<uint32_t, TxRecord> gTxMap;
static std::deque<uint32_t> gSeqOrder;    // insertion order, used to evict old entries
static size_t gTxMapMax = 20000;          // upper bound on buffered records

// kept for CSV inspection only (no longer used for verification)
std::vector<std::pair<float,float>> lastTransmittedSymbols;

// ----------------- QAM-16 constellation -----------------
std::pair<int,int> qam16_map[16] = {
  {-3,-3},{-3,-1},{-3, 3},{-3, 1},
  {-1,-3},{-1,-1},{-1, 3},{-1, 1},
  { 3,-3},{ 3,-1},{ 3, 3},{ 3, 1},
  { 1,-3},{ 1,-1},{ 1, 3},{ 1, 1}
};

uint8_t SecretBitsFromConstellation(int x0, int y0) {
  if ((x0 > 0) && (y0 > 0)) return 0b00;
  if ((x0 < 0) && (y0 > 0)) return 0b01;
  if ((x0 < 0) && (y0 < 0)) return 0b11;
  if ((x0 > 0) && (y0 < 0)) return 0b10;
  return 0xFF;
}

// ----------- Ellipses -----------
// Ellipse expression e(x, y) evaluated relative to constellation point (x0, y0)
float EllipseValue(float x, float y, int x0, int y0) {
  float e = 0.0f;
  if (x0 == -3 && y0 == -3)      e = ((-x + y)    *(-x + y)   )/40.0f + ((x + y + 6)  *(x + y + 6) )/80.0f;
  else if (x0 == -3 && y0 == -1) e = ((-x + y -2) *(-x + y -2))/40.0f + ((x + y + 4)  *(x + y + 4) )/80.0f;
  else if (x0 == -3 && y0 ==  3) e = ((-x - y)    *(-x - y)   )/40.0f + ((x - y + 6)  *(x - y + 6) )/80.0f;
  else if (x0 == -3 && y0 ==  1) e = ((-x - y -2) *(-x - y -2))/40.0f + ((x - y + 4)  *(x - y + 4) )/80.0f;
  else if (x0 == -1 && y0 == -3) e = ((-x + y +2) *(-x + y +2))/40.0f + ((x + y + 4)  *(x + y + 4) )/80.0f;
  else if (x0 == -1 && y0 == -1) e = ((-x + y)    *(-x + y)   )/40.0f + ((x + y + 2)  *(x + y + 2) )/80.0f;
  else if (x0 == -1 && y0 ==  3) e = ((-x - y +2) *(-x - y +2))/40.0f + ((x - y + 4)  *(x - y + 4) )/80.0f;
  else if (x0 == -1 && y0 ==  1) e = ((-x - y)    *(-x - y)   )/40.0f + ((x - y + 2)  *(x - y + 2) )/80.0f;
  else if (x0 ==  3 && y0 == -3) e = ((x + y)     *(x + y)    )/40.0f + ((-x + y + 6) *(-x + y + 6))/80.0f;
  else if (x0 ==  3 && y0 == -1) e = ((x + y -2)  *(x + y -2) )/40.0f + ((-x + y + 4) *(-x + y + 4))/80.0f;
  else if (x0 ==  3 && y0 ==  3) e = ((-x + y)    *(-x + y)   )/40.0f + ((x + y - 6)  *(x + y - 6) )/80.0f;
  else if (x0 ==  3 && y0 ==  1) e = ((-x + 2 + y)*(-x + 2 + y))/40.0f + ((x + y - 4) *(x + y - 4))/80.0f;
  else if (x0 ==  1 && y0 == -3) e = ((x + y + 2) *(x + y + 2))/40.0f + ((-x + y + 4) *(-x + y + 4))/80.0f;
  else if (x0 ==  1 && y0 == -1) e = ((x + y)     *(x + y)    )/40.0f + ((-x + y + 2) *(-x + y + 2))/80.0f;
  else if (x0 ==  1 && y0 ==  3) e = ((-x + y -2) *(-x + y -2))/40.0f + ((x + y - 4)  *(x + y - 4) )/80.0f;
  else if (x0 ==  1 && y0 ==  1) e = ((-x + y)    *(-x + y)   )/40.0f + ((x + y - 2)  *(x + y - 2) )/80.0f;
  else e = 1e9f; // fallback for an invalid constellation point
  return e;
}

// Smallest ellipse (closest to the point): decoder threshold
bool IsPointOutsideInnerEllipse(float x, float y, int x0, int y0) {
  float e = EllipseValue(x, y, x0, y0);
  return e > gEllipseE1; // previously a fixed 1/300
}

// Medium ellipse (currently unused, kept for experiments)
bool IsPointOutsideMediumEllipse(float x, float y, int x0, int y0) {
  float e = EllipseValue(x, y, x0, y0);
  return e > gEllipseE2;
}

// Largest ellipse: outer boundary only (useful for debugging)
bool IsPointOutsideOuterEllipse(float x, float y, int x0, int y0) {
  float e = EllipseValue(x, y, x0, y0);
  return e > gEllipseE3;
}


// ------------- Random secret bit source -------------
inline void EnsureSecretBits(size_t need) {
  static std::bernoulli_distribution coin(0.5);
  while (secretBitQueue.size() < need) {
    secretBitQueue.push(coin(rng));
  }
}

// ------------- Modulation with embedding -------------
std::vector<std::pair<float,float>> ModulateQam16WithStego(const std::vector<uint8_t>& data, bool logToCsv) {
  std::vector<std::pair<float,float>> symbols;
  for (uint8_t byte : data) {
    for (int shift = 4; shift >= 0; shift -= 4) {
      uint8_t nibble = (byte >> shift) & 0x0F;
      auto [x0, y0] = qam16_map[nibble];
      float x = x0, y = y0;

      uint8_t quadrant = SecretBitsFromConstellation(x0,y0);
      EnsureSecretBits(2);
      bool b1 = secretBitQueue.front(); secretBitQueue.pop();
      bool b2 = secretBitQueue.front(); secretBitQueue.pop();
      uint8_t bits = (b1 << 1) | b2;

    if (bits == quadrant) {
      // Sample candidates from a wide square around the constellation point
      std::uniform_real_distribution<float> dx(x0 - 2.0f, x0 + 2.0f);
      std::uniform_real_distribution<float> dy(y0 - 2.0f, y0 + 2.0f);

      for (int tries = 0; tries < 200; ++tries) {
        float tx = dx(rng), ty = dy(rng);

        // Ellipse value of the candidate point
        float e = EllipseValue(tx, ty, x0, y0);

        // Accept only points inside the embedding ring, i.e. between
        // the medium (gEllipseE2) and the largest (gEllipseE3) ellipse
        if (e > gEllipseE2 && e <= gEllipseE3) {
          x = tx; 
          y = ty;
          secretMessageSent.push_back(b1);
          secretMessageSent.push_back(b2);
          secretBitsSent += 2;
          break;
        }
      }
    } else {
        secretBitQueue.emplace(b1);
        secretBitQueue.emplace(b2);
      }
      if (logToCsv) {
        csvFile << "sent," << x << "," << y << "\n";
      }      
      symbols.emplace_back(x,y);
    }
  }
  return symbols;
}

// ------------- EVM from SNR [dB] -------------
inline double SnrDbToEvm(double snrDb) { return std::pow(10.0, -snrDb/20.0); }

// ------------- TX: periodic send tick -------------
double   gPktInterval = 0.010;
Ptr<Socket> gSendSock;
Address     gRemote;
static EventId gTickEvent;
static bool    gRunTicks = true;

static inline void PushTxMap(uint32_t seq, TxRecord&& rec) {
  gTxMap.emplace(seq, std::move(rec));
  gSeqOrder.push_back(seq);
  // evict the oldest records to bound memory usage
  while (gTxMap.size() > gTxMapMax) {
    uint32_t old = gSeqOrder.front(); gSeqOrder.pop_front();
    gTxMap.erase(old);
  }
}

void SendQamPacketOnce() {
  // Reserve the first 4 bytes of the payload for the sequence number
  if (kDataSize < 4) kDataSize = 4;
  const uint32_t payloadUserBytes = kDataSize - 4;

  std::uniform_int_distribution<int> byteRnd(0,255);
  std::vector<uint8_t> rawUser(payloadUserBytes);
  for (auto &b : rawUser) b = static_cast<uint8_t>(byteRnd(rng));


  // Modulate the user data; only the first packet is written to the CSV
    bool logThisPacketToCsv = gLogFirstPacketTx;
    std::vector<std::pair<float,float>> symbols = ModulateQam16WithStego(rawUser, logThisPacketToCsv);
    gLogFirstPacketTx = false;   // stop TX logging after the first packet


  // Build the full payload: [SEQ (4 B, little-endian)] + user data
  uint32_t seq = gSeq++;
  std::vector<uint8_t> full(kDataSize);
  std::memcpy(full.data(), &seq, sizeof(seq));
  std::memcpy(full.data()+4, rawUser.data(), rawUser.size());

  // store the record for verification on reception
  TxRecord rec;
  rec.data    = std::move(rawUser);
  rec.symbols = symbols;
  PushTxMap(seq, std::move(rec));

  // last packet's symbols, kept for inspection only
  lastTransmittedSymbols = symbols;

  Ptr<Packet> pkt = Create<Packet>(full.data(), full.size());
  gSendSock->SendTo(pkt, 0, gRemote);
  ++totalPackets;
}

void TrafficTick() {
  if (!gRunTicks) return;

  // send exactly one packet
  SendQamPacketOnce();

  // schedule the next packet after kSendIntervalSec
  gTickEvent = Simulator::Schedule(Seconds(kSendIntervalSec), &TrafficTick);
}

// ------------- UDP receive -------------
void ReceivePacket(Ptr<Socket> socket) {
  Address from;
  Ptr<Packet> pkt = socket->RecvFrom(from);
  std::vector<uint8_t> rx(pkt->GetSize());
  pkt->CopyData(rx.data(), rx.size());

  if (rx.size() < 4) return; // no sequence number; ignore (should not happen)
  uint32_t seq = 0;
  std::memcpy(&seq, rx.data(), sizeof(seq));
  // only the first packet (seq = 0) is written to the CSV
    bool logThisPacketToCsv = gLogFirstPacketRx && (seq == 0);


  auto it = gTxMap.find(seq);
  if (it == gTxMap.end()) {
    // record already evicted from the buffer; BER cannot be computed, skip
    return;
  }

  const auto &sym   = it->second.symbols;
  const auto &txdat = it->second.data;

  std::vector<uint8_t> demod;
  std::vector<bool> bits;

  const double evm   = SnrDbToEvm(gSnrDb);
  const double sigma = evm / std::sqrt(2.0);
  std::normal_distribution<double> gauss(0.0, sigma);

  for (auto [x0,y0] : sym) {
    float x = x0 + static_cast<float>(gauss(rng));
    float y = y0 + static_cast<float>(gauss(rng));

    float best = FLT_MAX; int idx = 0;
    for (int j=0;j<16;++j) {
      auto [cx,cy] = qam16_map[j];
      float dx = x - cx, dy = y - cy;
      float d2 = dx*dx + dy*dy;
      if (d2 < best) { best = d2; idx = j; }
    }
    auto [cx,cy] = qam16_map[idx];
    if (logThisPacketToCsv) {
        csvFile << "received," << x << "," << y << "\n";
      }
      

    bool inSquare = (x >= cx - 1.0f && x <= cx + 1.0f &&
                     y >= cy - 1.0f && y <= cy + 1.0f);

    // the decoder uses the smallest ellipse as its threshold
    if (inSquare && IsPointOutsideInnerEllipse(x, y, cx, cy)) {
      uint8_t q = SecretBitsFromConstellation(cx,cy);
      bool b1 = (q>>1)&1, b2 = q&1;
      secretMessageReceived.push_back(b1);
      secretMessageReceived.push_back(b2);
      secretBitsDetected += 2;
    }


    for (int b=3;b>=0;--b) bits.push_back((idx>>b)&1);
  }

  for (size_t i=0; i + 7 < bits.size(); i += 8) {
    uint8_t byte = 0;
    for (int b=0;b<8;++b) byte |= (bits[i+b] << (7-b));
    demod.push_back(byte);
  }

  totalBits += txdat.size()*8ULL;
  uint32_t localErrors = 0;
  size_t n = std::min(txdat.size(), demod.size());
  for (size_t i=0;i<n;++i) {
    uint8_t diff = txdat[i] ^ demod[i];
    for (int b=0;b<8;++b) if (diff & (1u<<b)) ++localErrors;
  }
  errorBits += localErrors;
  if (localErrors > 0) ++errorPackets;

  if (logThisPacketToCsv) {
    gLogFirstPacketRx = false;   // stop RX logging after the first packet
  }

  // record processed; release it
  gTxMap.erase(it);
}

// ----------- Airtime model -----------
double AirtimeUs(uint32_t payloadBytes) {
  double dataUs = ( (payloadBytes*8.0) / gAirMbps ) * 1e6;
  return gPlcpUs + gDifsUs + gAvgBackoffUs + dataUs + gSifsUs + gAckUs;
}

// =================== main ===================
int main (int argc, char** argv) {
  CommandLine cmd(__FILE__);
  cmd.AddValue("snrDb",   "SNR in dB for EVM-based noise", gSnrDb);
  cmd.AddValue("sim",     "Simulation time (s)", gSimSeconds);
  cmd.AddValue("airMbps", "Net PHY rate (Mbps) for payload", gAirMbps);
  cmd.AddValue("data",    "UDP payload bytes (total, incl. 4B seq)", kDataSize);
  cmd.AddValue("bg",      "Number of background STAs", gNumBgSta);
  cmd.AddValue("e1", "Ellipse threshold e1 (smallest, decoder)", gEllipseE1);
  cmd.AddValue("e2", "Ellipse threshold e2 (medium)", gEllipseE2);
  cmd.AddValue("e3", "Ellipse threshold e3 (largest)", gEllipseE3);

  cmd.Parse(argc, argv);
  rng.seed(time(NULL));



    // --- constellation output file (depends on SNR) ---
// --- output directory named after e1/e2/e3 ---

    std::ostringstream folderName;
    folderName << "all_constellation/"
               << gEllipseE1 << "_"
               << gEllipseE2 << "_"
               << gEllipseE3;

    // create the directory (recursively)
    std::ostringstream mkdirCmd;
    mkdirCmd << "mkdir -p " << folderName.str();
    int ret = system(mkdirCmd.str().c_str());
    (void)ret; // return code intentionally ignored


    // full path to the CSV file
    std::ostringstream constName;
    constName << folderName.str() << "/all_constellation_"
              << static_cast<int>(std::round(gSnrDb)) << ".csv";

    // open the file
    csvFile.open(constName.str());
    if (!csvFile.is_open()) {
        std::cerr << "Cannot open file: " << constName.str() << "\n";
    }
    csvFile << "type,x,y\n";




  // ----------------- Nodes -----------------
  NodeContainer apNode; apNode.Create(1);
  NodeContainer covertSta; covertSta.Create(1);
  NodeContainer bgStas; bgStas.Create(gNumBgSta);

  // Mobility
  MobilityHelper mob;
  mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mob.Install(apNode); mob.Install(covertSta); mob.Install(bgStas);
  apNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0.0,0.0,0.0));
  covertSta.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(5.0,0.0,0.0));
  for (uint32_t i=0;i<gNumBgSta;i++){
    bgStas.Get(i)->GetObject<MobilityModel>()->SetPosition(Vector(4.0 + 0.5*i, 1.0*(i%3), 0.0));
  }

  // Internet stack
  InternetStackHelper internet; internet.Install(apNode); internet.Install(covertSta); internet.Install(bgStas);

  // Channel + PHY
  YansWifiChannelHelper chan; chan.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  chan.AddPropagationLoss("ns3::LogDistancePropagationLossModel");
  YansWifiPhyHelper phy; phy.SetChannel(chan.Create());
  phy.Set("TxPowerStart", DoubleValue(30.0));
  phy.Set("TxPowerEnd", DoubleValue(30.0));
  phy.Set("TxPowerLevels", UintegerValue(1));
  phy.Set("Antennas", UintegerValue(1));
  phy.Set("ChannelSettings", StringValue("{1, 20, BAND_2_4GHZ, 0}")); // 2.4 GHz, 20 MHz

  Config::SetDefault("ns3::WifiMac::QosSupported", BooleanValue(true));
  WifiHelper wifi; wifi.SetStandard(WIFI_STANDARD_80211ax);
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
    "DataMode", StringValue("HeMcs3"),
    "ControlMode", StringValue("HeMcs0"));

  WifiMacHelper mac;
  Ssid ssid = Ssid("hybrid-ssid");

  // AP
  mac.SetType("ns3::ApWifiMac","Ssid", SsidValue(ssid));
  NetDeviceContainer apDev = wifi.Install(phy, mac, apNode.Get(0));

  // Covert STA
  mac.SetType("ns3::StaWifiMac","Ssid", SsidValue(ssid),"ActiveProbing", BooleanValue(false));
  NetDeviceContainer staDev = wifi.Install(phy, mac, covertSta.Get(0));

  // Background STAs (associated, no traffic)
  NetDeviceContainer bgStaDevs;
  for (uint32_t i=0;i<gNumBgSta;i++){
    NetDeviceContainer one = wifi.Install(phy, mac, bgStas.Get(i));
    bgStaDevs.Add(one);
  }

  // PHY timing (slot / SIFS)
  Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/Slot", TimeValue(MicroSeconds(gSlotUs)));
  Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/Sifs", TimeValue(MicroSeconds(gSifsUs)));

  // Addressing
  NetDeviceContainer allWifiDevices; allWifiDevices.Add(staDev); allWifiDevices.Add(apDev); allWifiDevices.Add(bgStaDevs);
  Ipv4AddressHelper ipv4; ipv4.SetBase("10.0.0.0","255.255.255.0");
  Ipv4InterfaceContainer ifaces = ipv4.Assign(allWifiDevices);
  Ipv4Address apAddr = ifaces.GetAddress(1); // AP

  // Sockets
  TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
  Ptr<Socket> rxSock = Socket::CreateSocket(apNode.Get(0), tid);
  rxSock->Bind(InetSocketAddress(Ipv4Address::GetAny(), kUdpPort));
  rxSock->SetRecvCallback(MakeCallback(&ReceivePacket));

  gSendSock = Socket::CreateSocket(covertSta.Get(0), tid);
  gRemote   = InetSocketAddress(apAddr, kUdpPort);

  // estimated airtime per frame (informational only)
  gPktInterval = AirtimeUs(kDataSize + gMacHdrBytes) / 1e6;
  if (!std::isfinite(gPktInterval) || gPktInterval <= 0.0) gPktInterval = 0.001;
  NS_LOG_UNCOND("airtime interval=" << gPktInterval
                 << " s (info) -> sending 1 packet every " << kSendIntervalSec << " s");

  // start traffic after 2 s (once association has completed)
  Simulator::Schedule(Seconds(2.0), &TrafficTick);

  Simulator::Stop(Seconds(gSimSeconds));
  Simulator::Run();
  Simulator::Destroy();

  // results
  double qamPer = totalPackets ? static_cast<double>(errorPackets) / static_cast<double>(totalPackets) : 0.0;
  double qamBer = totalBits    ? static_cast<double>(errorBits)    / static_cast<double>(totalBits)    : 0.0;

  std::cout << "\n=== QAM-16 RESULTS ===\n";
  std::cout << "Packets: " << totalPackets << ", errored: " << errorPackets
            << " -> PER = " << qamPer << "\n";
  std::cout << "Bits: " << totalBits << ", errored: " << errorBits
            << " -> BER = " << qamBer << "\n";
  std::cout << "Hidden bits sent: " << secretBitsSent << ", detected: " << secretBitsDetected << "\n";

  size_t L = std::min(secretMessageSent.size(), secretMessageReceived.size());
  uint32_t hiddenErr = 0;
  for (size_t i = 0; i < L; ++i)
  {
    if (secretMessageSent[i] != secretMessageReceived[i])
    {
      ++hiddenErr;
    }
  }
  double hiddenBer = L ? static_cast<double>(hiddenErr) / static_cast<double>(L) : 0.0;

  std::cout << "Hidden bit errors: " << hiddenErr << "/" << L
            << " -> hidden BER = " << hiddenBer << "\n";

  // --- append metrics to data_s1.csv ---
    {
      bool needHeader = true;
      {
        std::ifstream check("data_s1.csv");
        if (check.good())
        {
          // write the header only if the file is missing or empty
          if (check.peek() != std::ifstream::traits_type::eof())
          {
            needHeader = false;
          }
        }
      }

      std::ofstream phy("data_s1.csv", std::ios::app);
      if (!phy)
      {
        std::cerr << "Cannot open data_s1.csv for writing!\n";
      }
      else
      {
        if (needHeader)
        {
          // header includes the ellipse thresholds e1, e2, e3
          phy << "snr_db,qam_per,qam_ber,hidden_ber,e1,e2,e3\n";
        }
        // ellipse thresholds are logged with each result row
        phy << gSnrDb << "," 
            << qamPer << "," 
            << qamBer << "," 
            << hiddenBer << ","
            << gEllipseE1 << ","
            << gEllipseE2 << ","
            << gEllipseE3 << "\n";
      }
    }


  csvFile.close();
  return 0;
}