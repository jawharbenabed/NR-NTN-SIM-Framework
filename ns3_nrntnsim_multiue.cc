// ============================================================================
// ns3_nrntnsim_multiue.cc
// Multi-UE extension of approach5 — NR-NTN-SIM MSWiM 2026
// Supports N UEs (--numUEs), Jain fairness index, per-UE metrics
// Approche 5 — TNSM : Fix antenne UE VSAT Ka-band (CosineAntennaModel)
// ─────────────────────────────────────────────────────────────────────────────
// MODIFICATIONS vs optionD.cc :
//   Fix A (L~613) : UE AntennaElement → CosineAntennaModel MaxGain=26.7 dBi
//                   G_rx total = 12 dBi(4×4 array) + 26.7 dBi(elem) = 38.7 dBi
//                   Équivalent VSAT 0.6m Ka-band | TR 38.821 Table 6.3-1
//                   POURQUOI CosineAntennaModel :
//                     ThreeGppAntennaModel en ns-3.46 n'expose pas MaxGain
//                     comme attribut configurable → NS_FATAL au runtime
//                     CosineAntennaModel expose MaxGain depuis ns-3.30 ✓
//   Fix B (L~544) : Bande commentée — choisir 400 MHz (feeder) ou 50 MHz (UE)
//   Fix C (L~699) : lambdaDl + lambdaUl comme paramètres CMD (--lambdaDl, --lambdaUl)
//   Fix 1 (L~200) : MCS logging — mcs_derived (SE-based) + mcs_scheduled (gNB trace)
//   Fix 2 (L~748) : lambdaUl 2000 → 20000 (200 Mbps, saturation canal UL)
// ─────────────────────────────────────────────────────────────────────────────
// Résultats attendus après Fix A (400 MHz) :
//   600km/40dBm  : SINR -0.97 dB → CQI 5  → G_eff ~303 Mbps
//   600km/55dBm  : SINR 14.03 dB → CQI 15 → G_eff ~1909 Mbps (théorique)
//   1200km/55dBm : SINR  8.01 dB → CQI 11 → G_eff ~1142 Mbps (théorique)
// ─────────────────────────────────────────────────────────────────────────────
// Validation (run 30s) :
//   ./ns3 run ns3_nrntnsim_timeseries_optionD_approach5
//     satAlt=600 txPower=55 simTime=30
//     --outputDir=/tmp/approach5_validation/"
//   Checks : SINR > +12 dB | CQI in {13,14,15} | DL > 500 Mbps
// ============================================================================
#include "ns3/applications-module.h"
#include "ns3/antenna-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/geocentric-constant-position-mobility-model.h"
#include <fstream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <vector>
#include <string>
#include <map>
#include <signal.h>
#include <filesystem>
#include <sstream>
#include <set>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("Ns3NrNtnTimeSeriesOptionD");

// ============================================================================
// PARAMETERS GLOBAUX
// ============================================================================
static double   g_satAltKm      = 1200.0;
static double   g_txPowerDbm    = 60.0;
static double   g_simTime       = 30.0;
static double   g_tickIntervalS = 0.1;
static double   g_gslDelayMs    = 0.0;
// Paramètres trafic (overridables via CMD pour calibration par scénario)
static uint32_t g_numUEs   = 1;       // number of UEs
static uint32_t g_lambdaDl = 172564;  // pkt/s — 1728 Mbps offert DL (calibré 600/55)
static uint32_t g_lambdaUl = 20000;   // pkt/s — 200  Mbps offert UL (saturation UL)
static double   g_islDelayMs    = 0.0;
// ── Hypatia integration ──────────────────────────────────────────────────
static std::string g_hypatiaDir  = "";        // path to fstate files
static bool        g_hypatiaEnabled = false;
static double      g_hypatiaE2eMs   = 0.0;   // current E2E delay from Hypatia
static int         g_hypatiaHops    = 0;      // current ISL hop count
static std::map<std::pair<int,int>, std::tuple<int,int,int>> g_fstate; // (src,dst)->(nxt,isl,gsl)
static std::vector<int64_t> g_fstateTimestamps;  // sorted list of fstate timestamps (ns)
static size_t      g_fstateIdx = 0;           // index into g_fstateTimestamps
// ─────────────────────────────────────────────────────────────────────────
static uint32_t g_numIslHops    = 5;
static uint32_t g_tickId        = 0;

static const uint16_t PORT_UL_DATA = 1244;
static const uint16_t PORT_DL_DATA = 1234;
static const uint16_t PORT_ECHO    = 5001;

// ============================================================================
// FILES
// ============================================================================
static std::ofstream g_csvFile;
static std::ofstream g_rttFile;

// ============================================================================
// FLOWMONITOR
// ============================================================================
static Ptr<FlowMonitor>          g_monitor;
static Ptr<Ipv4FlowClassifier>   g_classifier;

struct FlowPrevStats
{
    uint64_t txBytes    = 0;
    uint64_t rxBytes    = 0;
    uint32_t txPackets  = 0;
    uint32_t rxPackets  = 0;
    uint64_t delaySumNs = 0;
    uint64_t jitterSumNs= 0;
};
static std::map<uint32_t, FlowPrevStats> g_prevFlowStats;

struct TickMetrics
{
    double   throughputMbps  = 0.0;
    double   delayMs         = -1.0;
    double   jitterMs        = -1.0;
    double   pdr             = -1.0;
    double   lossRate        = -1.0;
    uint64_t rxBytesTotal    = 0;
    uint32_t rxPacketsTotal  = 0;
    uint64_t txBytesTotal    = 0;
    uint32_t txPacketsTotal  = 0;
    uint64_t rxBytesDelta    = 0;
    uint32_t rxPacketsDelta  = 0;
    uint64_t txBytesDelta    = 0;
    uint32_t txPacketsDelta  = 0;
    uint64_t delaySumDeltaNs = 0;
    uint64_t jitterSumDeltaNs= 0;
};
static TickMetrics g_ulMetrics;
static TickMetrics g_dlMetrics;

// ============================================================================
// RTT
// ============================================================================
static double g_lastRttMs      = -1.0;
static bool   g_rttValid       = false;
static Time   g_lastEchoSendTime;

// ============================================================================
// PHY
// ============================================================================
static double   g_sinrDlSum   = 0.0;
static uint32_t g_sinrDlCount = 0;
static double   g_rsrpSum     = 0.0;
static uint32_t g_rsrpCount   = 0;
static int      g_lastCqi     = -1;
static int      g_derivedMcsDl = -1;   // MCS dérivé du débit mesuré (proxy CQI)
static uint8_t  g_scheduledMcs = 255;  // MCS schedulé via trace gNB (255=inconnu)

// Bande passante système (pour dériver MCS depuis SE mesurée)
static const double BW_MHZ_SIM = 400.0;

// Table SE→MCS (3GPP TS 38.214 Table 5.2.2.1-3, spectral efficiency bps/Hz)
static const double MCS_SE_TABLE[] = {
    0.15, 0.23, 0.38, 0.60, 0.88, 1.18, 1.48, 1.91, 2.41, 2.73,
    3.32, 3.90, 4.52, 5.12, 5.55, 5.55, 6.00, 6.51, 7.00, 7.45,
    7.90, 8.30, 8.70, 9.05, 9.40, 9.70, 10.0, 10.3, 10.6
};
static const int MCS_TABLE_SIZE = 29;

// ============================================================================
// HELPERS
// ============================================================================
enum FlowType { FLOW_UNKNOWN=0, FLOW_UL_DATA, FLOW_DL_DATA, FLOW_ECHO };

FlowType ClassifyFlow(const Ipv4FlowClassifier::FiveTuple& t)
{
    if (t.destinationPort == PORT_UL_DATA) return FLOW_UL_DATA;
    if (t.destinationPort == PORT_DL_DATA) return FLOW_DL_DATA;
    if (t.destinationPort == PORT_ECHO)    return FLOW_ECHO;
    return FLOW_UNKNOWN;
}

void ResetTickMetrics(TickMetrics& m) { m = TickMetrics{}; }

void AccumulateFlowToMetrics(TickMetrics& m,
                              const FlowMonitor::FlowStats& fs,
                              const FlowPrevStats& prev)
{
    uint64_t rxBytesDelta    = fs.rxBytes   - prev.rxBytes;
    uint64_t txBytesDelta    = fs.txBytes   - prev.txBytes;
    uint32_t rxPacketsDelta  = fs.rxPackets - prev.rxPackets;
    uint32_t txPacketsDelta  = fs.txPackets - prev.txPackets;
    uint64_t delaySumNs  = fs.delaySum.GetNanoSeconds()  >= prev.delaySumNs
                           ? fs.delaySum.GetNanoSeconds()  - prev.delaySumNs  : 0;
    uint64_t jitterSumNs = fs.jitterSum.GetNanoSeconds() >= prev.jitterSumNs
                           ? fs.jitterSum.GetNanoSeconds() - prev.jitterSumNs : 0;
    m.rxBytesTotal    += fs.rxBytes;
    m.rxPacketsTotal  += fs.rxPackets;
    m.txBytesTotal    += fs.txBytes;
    m.txPacketsTotal  += fs.txPackets;
    m.rxBytesDelta    += rxBytesDelta;
    m.rxPacketsDelta  += rxPacketsDelta;
    m.txBytesDelta    += txBytesDelta;
    m.txPacketsDelta  += txPacketsDelta;
    m.delaySumDeltaNs  += delaySumNs;
    m.jitterSumDeltaNs += jitterSumNs;
}

void FinalizeTickMetrics(TickMetrics& m)
{
    m.throughputMbps = (m.rxBytesDelta * 8.0) / (g_tickIntervalS * 1e6);
    m.delayMs  = (m.rxPacketsDelta > 0)
                 ? (static_cast<double>(m.delaySumDeltaNs) / m.rxPacketsDelta) / 1e6 : -1.0;
    m.jitterMs = (m.rxPacketsDelta > 1)
                 ? (static_cast<double>(m.jitterSumDeltaNs) / (m.rxPacketsDelta-1)) / 1e6 : -1.0;
    if (m.txPacketsDelta > 0) {
        m.pdr      = std::min(1.0, static_cast<double>(m.rxPacketsDelta) / m.txPacketsDelta);
        m.lossRate = 1.0 - m.pdr;
    } else {
        m.pdr = -1.0; m.lossRate = -1.0;
    }
}

// ============================================================================
// NR CALLBACKS
// ============================================================================
void RsrpCallback(std::string, uint16_t, uint16_t, double rsrp, double, bool isServing, uint8_t)
{
    if (isServing && rsrp > -200.0) { g_rsrpSum += rsrp; g_rsrpCount++; }
}
void DlPhyRxCallback(std::string, uint16_t, uint16_t, double sinr, uint16_t)
{
    if (sinr > 0.0) { g_sinrDlSum += 10.0 * std::log10(sinr); g_sinrDlCount++; }
}
void CqiCallback(std::string, uint16_t, uint16_t, uint8_t cqi)
{
    // Note : ne fire PAS en FDD 5G-LENA v4.2 → g_lastCqi reste -1
    // Le MCS réel est dérivé via g_derivedMcsDl dans WriteCsvRow
    g_lastCqi = static_cast<int>(cqi);
}
// ── FIX 1 — Scheduling trace côté gNB (MCS réel schedulé) ────────────────
void GnbSchedulingCallback(std::string context, uint32_t frame, uint32_t subframe,
                            uint32_t slot, uint32_t numSym, uint32_t tbSize,
                            uint8_t mcs, uint16_t rnti, uint8_t bwpId)
{
    if (bwpId == 0) // BWP0 = DL
        g_scheduledMcs = mcs;
}
void ConnectNrTraces()
{
    Config::ConnectFailSafe(
        "/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/DlDataSinr",
        MakeCallback(&DlPhyRxCallback));
    Config::ConnectFailSafe(
        "/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/DlCtrlSinr",
        MakeCallback(&DlPhyRxCallback));
    Config::ConnectFailSafe(
        "/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUeMac/DlCqiReport",
        MakeCallback(&CqiCallback));
    // ── FIX 1 — Traces scheduling gNB pour récupérer le MCS réel ──────────
    // NrGnbMac::DlScheduling : alternative si DlCqiReport ne fire pas en FDD
    Config::ConnectFailSafe(
        "/NodeList/*/DeviceList/*/ComponentCarrierMap/*/NrGnbMac/DlScheduling",
        MakeCallback(&GnbSchedulingCallback));
    Config::ConnectFailSafe(
        "/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/ReportUeMeasurements",
        MakeCallback(&RsrpCallback));
}

// ============================================================================
// RTT CALLBACKS
// ============================================================================
void EchoTxCallback(Ptr<const Packet>)
{
    g_lastEchoSendTime = Simulator::Now(); g_rttValid = false;
}
void EchoRxCallback(Ptr<const Packet>)
{
    Time now = Simulator::Now();
    if (g_lastEchoSendTime.GetNanoSeconds() > 0) {
        g_lastRttMs = (now - g_lastEchoSendTime).GetMilliSeconds();
        g_rttValid  = true;
        if (g_rttFile.is_open())
            g_rttFile << std::fixed << std::setprecision(4)
                      << now.GetSeconds() << "," << g_lastRttMs << ","
                      << g_satAltKm << "," << g_txPowerDbm << "\n";
    }
}

// ============================================================================
// FLOWMONITOR SAMPLING
// ============================================================================
void SampleFlowMonitor()
{
    if (!g_monitor || !g_classifier) return;
    ResetTickMetrics(g_ulMetrics);
    ResetTickMetrics(g_dlMetrics);
    g_monitor->CheckForLostPackets();
    auto stats = g_monitor->GetFlowStats();
    for (const auto& kv : stats) {
        uint32_t flowId = kv.first;
        const auto& fs  = kv.second;
        auto tuple      = g_classifier->FindFlow(flowId);
        FlowType type   = ClassifyFlow(tuple);
        FlowPrevStats prev;
        auto it = g_prevFlowStats.find(flowId);
        if (it != g_prevFlowStats.end()) prev = it->second;
        if      (type == FLOW_UL_DATA) AccumulateFlowToMetrics(g_ulMetrics, fs, prev);
        else if (type == FLOW_DL_DATA) AccumulateFlowToMetrics(g_dlMetrics, fs, prev);
        FlowPrevStats cur;
        cur.txBytes      = fs.txBytes;
        cur.rxBytes      = fs.rxBytes;
        cur.txPackets    = fs.txPackets;
        cur.rxPackets    = fs.rxPackets;
        cur.delaySumNs   = fs.delaySum.GetNanoSeconds();
        cur.jitterSumNs  = fs.jitterSum.GetNanoSeconds();
        g_prevFlowStats[flowId] = cur;
    }
    FinalizeTickMetrics(g_ulMetrics);
    FinalizeTickMetrics(g_dlMetrics);
}

// ============================================================================
// CSV
// ============================================================================
void InitCsv(const std::string& csvPath, const std::string& rttPath)
{
    g_csvFile.open(csvPath, std::ios::trunc);
    g_csvFile << "time_s,throughput_ul_mbps,throughput_dl_mbps,"
              << "delay_ul_ms,delay_dl_ms,jitter_ul_ms,jitter_dl_ms,"
              << "pdr_ul,pdr_dl,loss_rate_ul,loss_rate_dl,"
              << "rx_packets_ul_total,rx_packets_dl_total,"
              << "rx_bytes_ul_total,rx_bytes_dl_total,"
              << "tx_packets_ul_total,tx_packets_dl_total,"
              << "rtt_ms,rtt_valid,sinr_dl_db,rsrp_dbm,cqi,mcs_derived,mcs_scheduled,"
              << "traffic_dl_active,gsl_delay_ms,isl_delay_total_ms,"
              << "e2e_delay_theory_ms,sat_alt_km,tx_power_dbm,tick_id\n";
    g_rttFile.open(rttPath, std::ios::trunc);
    g_rttFile << "time_s,rtt_ms,sat_alt_km,tx_power_dbm\n";
}

void WriteCsvRow(double timeS)
{
    if (!g_csvFile.is_open()) return;
    double sinrDb  = (g_sinrDlCount > 0) ? (g_sinrDlSum / g_sinrDlCount) : -1.0;
    double rsrpDbm = (g_rsrpCount   > 0) ? (g_rsrpSum   / g_rsrpCount)   : -1.0;
    int    dlActive = (g_dlMetrics.rxBytesDelta > 0) ? 1 : 0;
    double islTotal = g_numIslHops * g_islDelayMs;
    double e2eMs    = 2.0 * g_gslDelayMs + islTotal;
    // ── FIX 1 — Dériver le MCS depuis la SE mesurée ──────────────────────────
    // SE = throughput_mbps / BW_MHz (bps/Hz brut, sans correction overhead)
    // On compare à la table 3GPP TS 38.214 pour trouver le MCS correspondant
    if (g_dlMetrics.throughputMbps > 0.0) {
        double se = g_dlMetrics.throughputMbps / BW_MHZ_SIM;
        g_derivedMcsDl = 0;
        for (int i = MCS_TABLE_SIZE - 1; i >= 0; i--) {
            if (se >= MCS_SE_TABLE[i]) { g_derivedMcsDl = i; break; }
        }
    } else {
        g_derivedMcsDl = -1;
    }
    g_csvFile << std::fixed << std::setprecision(4)
              << timeS                          << ","
              << g_ulMetrics.throughputMbps     << ","
              << g_dlMetrics.throughputMbps     << ","
              << g_ulMetrics.delayMs            << ","
              << g_dlMetrics.delayMs            << ","
              << g_ulMetrics.jitterMs           << ","
              << g_dlMetrics.jitterMs           << ","
              << g_ulMetrics.pdr                << ","
              << g_dlMetrics.pdr                << ","
              << g_ulMetrics.lossRate           << ","
              << g_dlMetrics.lossRate           << ","
              << g_ulMetrics.rxPacketsTotal     << ","
              << g_dlMetrics.rxPacketsTotal     << ","
              << g_ulMetrics.rxBytesTotal       << ","
              << g_dlMetrics.rxBytesTotal       << ","
              << g_ulMetrics.txPacketsTotal     << ","
              << g_dlMetrics.txPacketsTotal     << ","
              << (g_rttValid ? g_lastRttMs : -1.0) << ","
              << (g_rttValid ? 1 : 0)           << ","
              << sinrDb                         << ","
              << rsrpDbm                        << ","
              << g_lastCqi                      << ","
              << g_derivedMcsDl                   << ","
              << static_cast<int>(g_scheduledMcs)  << ","
              << dlActive                       << ","
              << g_gslDelayMs                   << ","
              << islTotal                       << ","
              << e2eMs                          << ","
              << g_satAltKm                     << ","
              << g_txPowerDbm                   << ","
              << g_tickId                       << "\n";
    g_sinrDlSum = 0.0; g_sinrDlCount = 0;
    g_rsrpSum   = 0.0; g_rsrpCount   = 0;
    g_rttValid  = false; g_lastRttMs = -1.0;
    g_scheduledMcs = 255;  // reset — 255 = aucun scheduling ce tick
}

// ============================================================================
// TICK
// ============================================================================
void OnTick()
{
    g_tickId++;
    double timeS = Simulator::Now().GetSeconds();
    SampleFlowMonitor();
    WriteCsvRow(timeS);
    if (timeS < g_simTime - g_tickIntervalS)
        Simulator::Schedule(Seconds(g_tickIntervalS), &OnTick);
}

// ============================================================================
// MAIN
// ============================================================================
// ════════════════════════════════════════════════════════════════════════════
// HYPATIA FSTATE READER
// ════════════════════════════════════════════════════════════════════════════
void HypatiaLoadFstate(const std::string& filepath)
{
    std::ifstream f(filepath);
    if (!f.is_open()) return;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string tok;
        std::vector<int> v;
        while (std::getline(ss, tok, ',')) v.push_back(std::stoi(tok));
        if (v.size() == 5) {
            g_fstate[{v[0], v[1]}] = {v[2], v[3], v[4]};
        }
    }
}

int HypatiaCountHops(int src_gs, int dst_gs, int num_sats)
{
    // Find satellite serving src_gs
    int src_sat = -1;
    for (auto& kv : g_fstate) {
        int s = kv.first.first, d = kv.first.second;
        auto [nxt, isl, gsl] = kv.second;
        if (d == src_gs && s < num_sats && nxt == src_gs) {
            src_sat = s;
            break;
        }
    }
    if (src_sat < 0) return -1;
    int current = src_sat, hops = 0;
    std::set<int> visited;
    while (current != dst_gs && hops < 20) {
        if (visited.count(current)) break;
        visited.insert(current);
        auto it = g_fstate.find({current, dst_gs});
        if (it == g_fstate.end()) break;
        auto [nxt, isl, gsl] = it->second;
        if (nxt == dst_gs) break;
        current = nxt;
        hops++;
    }
    return hops;
}

void HypatiaUpdate(int64_t sim_ns)
{
    if (!g_hypatiaEnabled || g_fstateTimestamps.empty()) return;
    // Apply all fstate files with timestamp <= sim_ns
    while (g_fstateIdx < g_fstateTimestamps.size() &&
           g_fstateTimestamps[g_fstateIdx] <= sim_ns) {
        std::ostringstream fname;
        fname << g_hypatiaDir << "/fstate_"
              << g_fstateTimestamps[g_fstateIdx] << ".txt";
        HypatiaLoadFstate(fname.str());
        g_fstateIdx++;
    }
    // Compute E2E delay: Paris(GS0=node66) -> Madrid(GS1=node67)
    const int NUM_SATS   = 66;
    const int SRC_GS     = NUM_SATS;      // node 66 = Paris
    const int DST_GS     = NUM_SATS + 1;  // node 67 = Madrid
    const double C_KMS   = 299792.458;
    const double ISL_KM  = 2.0 * (6371.0 + 1200.0) * M_PI / NUM_SATS;

    int hops = HypatiaCountHops(SRC_GS, DST_GS, NUM_SATS);
    if (hops >= 0) {
        g_hypatiaHops  = hops;
        double isl_ms  = hops * (ISL_KM / C_KMS) * 1000.0;
        g_hypatiaE2eMs = 2.0 * g_gslDelayMs + isl_ms;
        g_islDelayMs   = (hops > 0) ? (ISL_KM / C_KMS) * 1000.0 : 0.0;
    }
}

void HypatiaInit(const std::string& dir)
{
    g_hypatiaDir     = dir;
    g_hypatiaEnabled = true;
    // Scan directory for fstate_*.txt files
    namespace fs = std::filesystem;
    for (auto& entry : fs::directory_iterator(dir)) {
        std::string name = entry.path().filename().string();
        if (name.rfind("fstate_", 0) == 0 && name.size() > 11) {
            try {
                std::string ts_str = name.substr(7, name.size() - 11);
                int64_t ts = std::stoll(ts_str);
                g_fstateTimestamps.push_back(ts);
            } catch (...) {}
        }
    }
    std::sort(g_fstateTimestamps.begin(), g_fstateTimestamps.end());
    std::cout << "[HYPATIA] Loaded " << g_fstateTimestamps.size()
              << " fstate timestamps from " << dir << std::endl;
    // Apply t=0
    HypatiaUpdate(0);
    std::cout << "[HYPATIA] t=0 : " << g_hypatiaHops << " ISL hops"
              << ", E2E=" << g_hypatiaE2eMs << " ms" << std::endl;
}

void HypatiaScheduledUpdate()
{
    int64_t sim_ns = Simulator::Now().GetNanoSeconds();
    int prev_hops  = g_hypatiaHops;
    HypatiaUpdate(sim_ns);
    if (g_hypatiaHops != prev_hops) {
        std::cout << "[HYPATIA] t=" << sim_ns/1e9 << "s HANDOVER: "
                  << prev_hops << " -> " << g_hypatiaHops << " ISL hops"
                  << ", E2E=" << g_hypatiaE2eMs << " ms" << std::endl;
    }
    Simulator::Schedule(MilliSeconds(100), &HypatiaScheduledUpdate);
}
// ════════════════════════════════════════════════════════════════════════════

int main(int argc, char* argv[])
{
    signal(SIGPIPE, SIG_IGN);

    std::string outputDir = std::string(getenv("HOME")) + "/phd_sim/metrics/";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime",   "Simulation time (s)",      g_simTime);
    cmd.AddValue("satAlt",    "Satellite altitude (km)",   g_satAltKm);
    cmd.AddValue("txPower",   "gNB TX power (dBm)",        g_txPowerDbm);
    cmd.AddValue("outputDir", "Output directory",          outputDir);
    std::string hypatiaDir = "";
    cmd.AddValue("hypatiaDir", "Path to Hypatia fstate directory (empty=disabled)", hypatiaDir);
    // ── Paramètres trafic (calibrés par scénario pour PDR > 90%) ──────────
    cmd.AddValue("numUEs",   "Number of UEs (1/2/4/8)",                g_numUEs);
    cmd.AddValue("lambdaDl", "DL offered traffic (pkt/s, 1pkt=1252B)", g_lambdaDl);
    cmd.AddValue("lambdaUl", "UL offered traffic (pkt/s, 1pkt=1252B)", g_lambdaUl);
    cmd.Parse(argc, argv);

    system(("mkdir -p " + outputDir).c_str());

    // Suppress verbose channel condition logs (Dense Urban NLOS spam)
    LogComponentDisable("ThreeGppChannelConditionModel", LOG_LEVEL_ALL);
    LogComponentDisable("ThreeGppPropagationLossModel",  LOG_LEVEL_ALL);
    LogComponentDisable("ThreeGppSpectrumPropagationLossModel", LOG_LEVEL_ALL);

    // Calcul délais NTN
    const double SPEED_OF_LIGHT_KMS = 3.0e5;
    const int    NUM_SATS            = 12;
    const double EARTH_RADIUS_KM    = 6371.0;
    double R         = EARTH_RADIUS_KM + g_satAltKm;
    double islDistKm = 2.0 * R * std::sin(M_PI / NUM_SATS);
    g_islDelayMs = (islDistKm / SPEED_OF_LIGHT_KMS) * 1000.0;
    g_gslDelayMs = (g_satAltKm / SPEED_OF_LIGHT_KMS) * 1000.0;
    // Hypatia integration : override ISL delay if directory provided
    if (!hypatiaDir.empty()) {
        HypatiaInit(hypatiaDir);
    }
    double totalBackhaulDelayMs = g_hypatiaEnabled
        ? g_hypatiaE2eMs
        : (2.0 * g_gslDelayMs + g_numIslHops * g_islDelayMs);

    std::cout << "[NTN] satAlt="    << g_satAltKm
              << "km txPower="      << g_txPowerDbm
              << "dBm GSL="         << g_gslDelayMs
              << "ms ISL="          << g_islDelayMs
              << "ms backhaul="     << totalBackhaulDelayMs << "ms\n";

    // CSV init
    std::ostringstream tag;
    tag << "alt" << static_cast<int>(g_satAltKm)
        << "_pwr" << static_cast<int>(g_txPowerDbm)
        << "_ue"  << g_numUEs;
    InitCsv(outputDir + "timeseries_" + tag.str() + ".csv",
            outputDir + "rtt_"        + tag.str() + ".csv");

    // ------------------------------------------------------------------------
    // Nodes
    // ------------------------------------------------------------------------
    NodeContainer ueNodes, gnbNodes;
    ueNodes.Create(g_numUEs);
    gnbNodes.Create(1);

    // Mobility UE (Paris, sol)
    MobilityHelper mobUe;
    mobUe.SetMobilityModel("ns3::GeocentricConstantPositionMobilityModel");
    mobUe.Install(ueNodes);
    // UEs near Paris — small offset (~11m) to avoid elevation angle assertion
    // between co-located nodes in ns-3 channel-condition-model
    for (uint32_t i = 0; i < g_numUEs; ++i) {
        double latOffset = 0.0001 * static_cast<double>(i);  // ~11m N
        double lonOffset = 0.0001 * static_cast<double>(i);  // ~8m E
        ueNodes.Get(i)->GetObject<GeocentricConstantPositionMobilityModel>()
            ->SetGeographicPosition(Vector(48.85 + latOffset,
                                           2.35 + lonOffset, 0.0));
    }

    // Mobility gNB (satellite au-dessus de Paris)
    MobilityHelper mobGnb;
    mobGnb.SetMobilityModel("ns3::GeocentricConstantPositionMobilityModel");
    mobGnb.Install(gnbNodes);
    gnbNodes.Get(0)->GetObject<GeocentricConstantPositionMobilityModel>()
        ->SetGeographicPosition(Vector(48.85, 2.35, g_satAltKm * 1000.0));

    // ------------------------------------------------------------------------
    // NR + EPC
    // ------------------------------------------------------------------------
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper>  beam      = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper>                nrHelper  = CreateObject<NrHelper>();
    Ptr<NrChannelHelper>         chHelper  = CreateObject<NrChannelHelper>();

    nrHelper->SetEpcHelper(epcHelper);
    nrHelper->SetBeamformingHelper(beam);

    // Canal NTN 3GPP — configure via CreateBandwidthParts
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(0)));

    // FDD NTN Ka-band : CreateBandwidthParts avec 2 BWPs (3GPP TR 38.821)
    // BWP0 = DL (satellite -> UE), BWP1 = UL (UE -> satellite)
    // FDD NTN Ka-band : deux CCs séparées (configuration manuelle CTTC)
    // CC0/BWP0 = DL 19.7 GHz 400 MHz  (satellite -> UE)
    // CC1/BWP1 = UL 29.5 GHz 400 MHz  (UE -> satellite)
    CcBwpCreator ccBwpCreator;
    OperationBandInfo band;
    band.m_centralFrequency = 24.6e9; // centre entre 19.7 et 29.5 GHz
    band.m_channelBandwidth = 19.6e9; // couvre les deux bandes
    band.m_lowerFrequency   = band.m_centralFrequency - band.m_channelBandwidth / 2;
    band.m_higherFrequency  = band.m_centralFrequency + band.m_channelBandwidth / 2;

    // CC0 : DL 19.7 GHz
    auto cc0 = std::make_unique<ComponentCarrierInfo>();
    cc0->m_ccId = 0;
    cc0->m_centralFrequency = 19.7e9;
    // ── FIX B (optionnel) ─────────────────────────────────────────────────────
    // 400e6 = lien feeder dédié (actuel) | 50e6 = lien accès UE (TR 38.821)
    // → Pour 50 MHz : remplacer 400e6 par 50e6 ici ET ligne ~559, ET lambdaDl→30000
    cc0->m_channelBandwidth = 400e6;   // feeder | accès UE : 50e6
    cc0->m_lowerFrequency   = cc0->m_centralFrequency - cc0->m_channelBandwidth / 2;
    cc0->m_higherFrequency  = cc0->m_centralFrequency + cc0->m_channelBandwidth / 2;
    auto bwp0 = std::make_unique<BandwidthPartInfo>();
    bwp0->m_bwpId = 0;
    bwp0->m_centralFrequency = cc0->m_centralFrequency;
    bwp0->m_channelBandwidth = cc0->m_channelBandwidth;
    bwp0->m_lowerFrequency   = cc0->m_lowerFrequency;
    bwp0->m_higherFrequency  = cc0->m_higherFrequency;
    cc0->AddBwp(std::move(bwp0));

    // CC1 : UL 29.5 GHz
    auto cc1 = std::make_unique<ComponentCarrierInfo>();
    cc1->m_ccId = 1;
    cc1->m_centralFrequency = 29.5e9;
    cc1->m_channelBandwidth = 400e6;   // feeder | accès UE : 50e6
    cc1->m_lowerFrequency   = cc1->m_centralFrequency - cc1->m_channelBandwidth / 2;
    cc1->m_higherFrequency  = cc1->m_centralFrequency + cc1->m_channelBandwidth / 2;
    auto bwp1 = std::make_unique<BandwidthPartInfo>();
    bwp1->m_bwpId = 1;
    bwp1->m_centralFrequency = cc1->m_centralFrequency;
    bwp1->m_channelBandwidth = cc1->m_channelBandwidth;
    bwp1->m_lowerFrequency   = cc1->m_lowerFrequency;
    bwp1->m_higherFrequency  = cc1->m_higherFrequency;
    cc1->AddBwp(std::move(bwp1));

    band.AddCc(std::move(cc0));
    band.AddCc(std::move(cc1));
    chHelper->ConfigureFactories("NTN-DenseUrban", "Default", "ThreeGpp");
    chHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    chHelper->AssignChannelsToBands({band});
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    // QoS flow -> BWP mapping
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(0));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB",  UintegerValue(0));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_CONV_VOICE",    UintegerValue(1));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_CONV_VOICE",     UintegerValue(1));

    beam->SetAttribute("BeamformingMethod",
        TypeIdValue(DirectPathBeamforming::GetTypeId()));

    // -------------------------------------------------------------------------
    // LINK BUDGET 3GPP TR 38.821 Ka-band NTN
    // -------------------------------------------------------------------------
    // gNB (satellite) TxPower DL : parametre variable (typique 40-55 dBm)
    nrHelper->SetGnbPhyAttribute("TxPower",      DoubleValue(g_txPowerDbm));
    // gNB NoiseFigure : satellite LNA Ka-band = 2 dB (TR 38.821 Table 6.3-1)
    nrHelper->SetGnbPhyAttribute("NoiseFigure",  DoubleValue(2.0));
    // UE (terminal VSAT) TxPower UL : 43 dBm max (TR 38.821 Table 6.3-1)
    nrHelper->SetUePhyAttribute("TxPower",       DoubleValue(43.0));
    // UE NoiseFigure : terminal Ka-band = 7 dB (TR 38.821 Table 6.3-1)
    nrHelper->SetUePhyAttribute("NoiseFigure",   DoubleValue(7.0));
    Config::SetDefault("ns3::NrUePowerControl::Pcmax",     DoubleValue(43.0));
    Config::SetDefault("ns3::NrUePowerControl::ClosedLoop", BooleanValue(false));
    // Buffer RLC agrandi pour NTN (grand RTT = buffer doit absorber le pipeline)
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(10 * 1024 * 1024));
    Config::SetDefault("ns3::NrRlcAm::MaxTxBufferSize", UintegerValue(10 * 1024 * 1024));
    // FDD : PrimaryUlIndex=1 AVANT InstallUeDevice (CHANGES.md)
    Config::SetDefault("ns3::NrUeNetDevice::PrimaryUlIndex", UintegerValue(1));

    // Antenne gNB satellite Ka-band : 32x32 = 1024 elements (APA satellite Ka-band)
    nrHelper->SetGnbAntennaAttribute("NumRows",    UintegerValue(32));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(32));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
        PointerValue(CreateObject<IsotropicAntennaModel>()));
    // UE antenne : 4x4 = 16 elements (terminal VSAT Ka-band TR 38.821)
    nrHelper->SetUeAntennaAttribute("NumRows",    UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    // ── FIX A — Approche 5 : antenne UE VSAT Ka-band ──────────────────────────
    // PROBLÈME : IsotropicAntennaModel → G_rx = 0 dBi par élément
    //            G_rx total = 10*log10(4×4) + 0 = 12 dBi seulement
    //            → SINR loggé sous-estimé de 26.7 dB → CQI = -1 partout
    //
    // SOLUTION : CosineAntennaModel avec MaxGain = 26.7 dBi
    //   Note : ThreeGppAntennaModel en ns-3.46 n'expose PAS MaxGain
    //          comme attribut TypeId → NS_FATAL au runtime → utiliser Cosine
    //   CosineAntennaModel : G(θ) = MaxGain × cos^n(θ/2)
    //   À boresight (θ=0) : G = MaxGain = 26.7 dBi
    //   Avec DirectPathBeamforming → beam vers satellite → θ ≈ 0° → G = 26.7 dBi ✓
    //   G_rx total = 12.0 dBi (4×4 array) + 26.7 dBi (elem) = 38.7 dBi
    //   ≈ terminal VSAT fixe Ka-band 0.6m (3GPP TR 38.821 Table 6.3-1)
    {
        auto ueAntElem = CreateObject<CosineAntennaModel>();
        ueAntElem->SetAttribute("MaxGain", DoubleValue(26.7)); // dBi — VSAT Ka-band 0.6m
        // Note : Beamwidth n'existe pas en ns-3.46 CosineAntennaModel
        // MaxGain seul suffit : DirectPathBeamforming → beam vers satellite → θ≈0° toujours
        nrHelper->SetUeAntennaAttribute("AntennaElement",
            PointerValue(ueAntElem));
    }
    nrHelper->SetSchedulerAttribute("FixedMcsDl", BooleanValue(false));

    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueNetDev  = nrHelper->InstallUeDevice(ueNodes,   allBwps);
    // BUG 5G-LENA v4.2 : PrimaryUlIndex stocké dans NrUeNetDevice mais jamais
    // transmis au NrUeRrc dans DoInitialize() — on le set directement sur le RRC
    for (uint32_t i = 0; i < ueNetDev.GetN(); ++i)
    {
        DynamicCast<NrUeNetDevice>(ueNetDev.Get(i))->GetRrc()->SetPrimaryUlIndex(1);
    }
    // FDD patterns : BWP0=DL only, BWP1=UL only
    NrHelper::GetGnbPhy(gnbNetDev.Get(0), 0)->SetAttribute("Pattern", StringValue("DL|DL|DL|DL|DL|DL|DL|DL|DL|DL|"));
    NrHelper::GetGnbPhy(gnbNetDev.Get(0), 1)->SetAttribute("Pattern", StringValue("UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|"));
    // FDD: RAR et messages DL ctrl de BWP1 (UL) redirigés vers BWP0 (DL)
    DynamicCast<NrGnbNetDevice>(gnbNetDev.Get(0))->GetBwpManager()->SetOutputLink(1, 0);
    // FDD: UE messages UL de BWP0 (DL) redirigés vers BWP1 (UL)
    for (uint32_t i = 0; i < g_numUEs; ++i) {
        DynamicCast<NrUeNetDevice>(ueNetDev.Get(i))->GetBwpManager()->SetOutputLink(0, 1);
    }

    // Internet stack sur UE - doit être après InstallUeDevice
    InternetStackHelper internet;
    internet.Install(ueNodes);

    int64_t randomStream = 1;
    randomStream += nrHelper->AssignStreams(gnbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueNetDev,  randomStream);

    // IP sur UE
    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(ueNetDev);
    nrHelper->AttachToClosestGnb(ueNetDev, gnbNetDev);
    nrHelper->UpdateDeviceConfigs(gnbNetDev);
    nrHelper->UpdateDeviceConfigs(ueNetDev);

    // Remote host avec délai NTN backhaul
    auto [remoteHost, remoteHostIpv4Address] =
        epcHelper->SetupRemoteHost("100Gb/s", 2500, MilliSeconds(totalBackhaulDelayMs));
    // Route PGW -> RemoteHost (1.0.0.0/8) manquante dans SetupRemoteHost
    // Sans cette route, les paquets UL apres de-encapsulation GTP sont droppes
    {
        Ptr<Node> pgw = epcHelper->GetPgwNode();
        Ptr<Ipv4> pgwIpv4 = pgw->GetObject<Ipv4>();
        Ipv4StaticRoutingHelper ipv4RoutingHelper;
        Ptr<Ipv4RoutingProtocol> rp = pgwIpv4->GetRoutingProtocol();
        Ptr<Ipv4ListRouting> lr = DynamicCast<Ipv4ListRouting>(rp);
        Ptr<Ipv4StaticRouting> sr;
        if (lr) {
            for (uint32_t k = 0; k < lr->GetNRoutingProtocols(); k++) {
                int16_t prio;
                sr = DynamicCast<Ipv4StaticRouting>(lr->GetRoutingProtocol(k, prio));
                if (sr) break;
            }
        }
        if (sr) {
            uint32_t nIf = pgwIpv4->GetNInterfaces();
            for (uint32_t i = 1; i < nIf; i++) {
                Ipv4Address addr = pgwIpv4->GetAddress(i, 0).GetLocal();
                if ((addr.Get() & 0xFF000000) == 0x01000000) { // 1.0.0.x
                    sr->AddNetworkRouteTo(Ipv4Address("1.0.0.0"),
                                         Ipv4Mask("255.0.0.0"), i);
                    std::cout << "[ROUTING] PGW route 1.0.0.0/8 via iface " << i << "\n";
                    break;
                }
            }
        }
    }

    std::cout << "[BACKHAUL] delay=" << totalBackhaulDelayMs
              << "ms  remoteHost=" << remoteHostIpv4Address << "\n";



    // ------------------------------------------------------------------------
    // Applications DL : RemoteHost -> UE
    // ------------------------------------------------------------------------
    Time udpAppStartTime = MilliSeconds(400);
    Time simTimeNs3      = Seconds(g_simTime);

    // Paramètres trafic
    uint32_t pktSize  = 1252;
    // Trafic offert >> capacite lien pour forcer variation MCS avec SINR
    // DL : ~200 Mbps offert (satellite -> UE) — adapte selon SINR/altitude
    // UL : ~2 Mbps offert (UE -> satellite) — asymetrie reelle NTN Ka-band
    // Trafic Ka-band NTN eMBB (3GPP TR 38.821) — ratio DL/UL = 10:1
    // DL : ~200 Mbps offert (satellite -> UE)
    // UL : ~20 Mbps offert (UE -> satellite) — terminal eMBB Ka-band realiste
    // ── Trafic DL — utilise g_lambdaDl (configurable par CMD) ───────────────
    // Valeur par défaut : 172564 pkt/s = 1728 Mbps (calibré pour 600/55 PDR>94%)
    // Calibration recommandée par scénario (voir run_all_scenarios.sh) :
    //   600/40  : 123000  (1231 Mbps ≈ G_eff×1.05)
    //   600/45  : 148000  (1482 Mbps)
    //   600/50  : 165000  (1652 Mbps)
    //   600/55  : 172564  (1728 Mbps) ← défaut
    //   1200/40 :  85000  ( 851 Mbps)
    //   1200/45 : 105000  (1051 Mbps)
    //   1200/50 : 137000  (1371 Mbps)
    //   1200/55 : 157000  (1571 Mbps)
    // Per-UE offered load — total load split equally across UEs (rho=0.90 preserved)
    uint32_t lambdaDl  = std::max(1u, g_lambdaDl  / g_numUEs);
    uint32_t lambdaUl  = std::max(1u, g_lambdaUl  / g_numUEs);
    std::cout << "[MULTI-UE] N=" << g_numUEs
              << "  lambdaDl/UE=" << lambdaDl
              << "  lambdaUl/UE=" << lambdaUl << "\n";

    ApplicationContainer serverApps, clientApps;

    // ── Per-UE DL + UL applications ──────────────────────────────────────
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(1));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(1));

    UdpServerHelper ulSinkGlobal(PORT_UL_DATA);
    serverApps.Add(ulSinkGlobal.Install(remoteHost));

    for (uint32_t i = 0; i < g_numUEs; ++i) {
        // DL: remoteHost -> UE_i:PORT_DL_DATA
        UdpServerHelper dlSink(PORT_DL_DATA);
        serverApps.Add(dlSink.Install(ueNodes.Get(i)));

        UdpClientHelper dlClient;
        dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        dlClient.SetAttribute("PacketSize", UintegerValue(pktSize));
        dlClient.SetAttribute("Interval",   TimeValue(Seconds(1.0 / lambdaDl)));
        dlClient.SetAttribute("Remote",
            AddressValue(addressUtils::ConvertToSocketAddress(
                ueIpIface.GetAddress(i), PORT_DL_DATA)));
        clientApps.Add(dlClient.Install(remoteHost));

        NrQosFlow dlFlow(NrQosFlow::NGBR_LOW_LAT_EMBB);
        Ptr<NrQosRule> dlRule = Create<NrQosRule>();
        NrQosRule::PacketFilter dlpf;
        dlpf.localPortStart = PORT_DL_DATA;
        dlpf.localPortEnd   = PORT_DL_DATA;
        dlpf.direction      = NrQosRule::DOWNLINK;
        dlRule->Add(dlpf);
        nrHelper->ActivateDedicatedQosFlow(ueNetDev.Get(i), dlFlow, dlRule);

        // UL: UE_i -> remoteHost:PORT_UL_DATA
        UdpClientHelper ulClient;
        ulClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        ulClient.SetAttribute("PacketSize", UintegerValue(pktSize));
        ulClient.SetAttribute("Interval",   TimeValue(Seconds(1.0 / lambdaUl)));
        ulClient.SetAttribute("Remote",
            AddressValue(addressUtils::ConvertToSocketAddress(
                remoteHostIpv4Address, PORT_UL_DATA)));
        clientApps.Add(ulClient.Install(ueNodes.Get(i)));

        NrQosFlow ulFlow(NrQosFlow::GBR_CONV_VOICE);
        Ptr<NrQosRule> ulRule = Create<NrQosRule>();
        NrQosRule::PacketFilter ulFilter;
        ulFilter.remotePortStart = PORT_UL_DATA;
        ulFilter.remotePortEnd   = PORT_UL_DATA;
        ulFilter.direction       = NrQosRule::UPLINK;
        ulRule->Add(ulFilter);
        nrHelper->ActivateDedicatedQosFlow(ueNetDev.Get(i), ulFlow, ulRule);
    }

    // ------------------------------------------------------------------------
    // Echo (RTT)
    // ------------------------------------------------------------------------
    UdpEchoServerHelper echoServer(PORT_ECHO);
    serverApps.Add(echoServer.Install(remoteHost));
    UdpEchoClientHelper echoClient(remoteHostIpv4Address, PORT_ECHO);
    echoClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    echoClient.SetAttribute("Interval",   TimeValue(Seconds(1.0)));
    echoClient.SetAttribute("PacketSize", UintegerValue(64));
    ApplicationContainer echoClientApps = echoClient.Install(ueNodes.Get(0));

    serverApps.Start(udpAppStartTime);
    clientApps.Start(udpAppStartTime);
    serverApps.Stop(simTimeNs3);
    clientApps.Stop(simTimeNs3);
    echoClientApps.Start(udpAppStartTime);
    echoClientApps.Stop(simTimeNs3);

    // ------------------------------------------------------------------------
    // FlowMonitor
    // ------------------------------------------------------------------------
    FlowMonitorHelper flowmonHelper;
    g_monitor    = flowmonHelper.InstallAll();
    g_classifier = DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());

    // ------------------------------------------------------------------------
    // NR Traces + Tick
    // ------------------------------------------------------------------------
    // PCAP debug temporaire — tracer le chemin UL
    PointToPointHelper p2pDebug;
    //p2pDebug.EnablePcapAll("/tmp/ntn_debug", true);
    Simulator::Schedule(Seconds(0.0), &ConnectNrTraces);
    if (g_hypatiaEnabled) {
        Simulator::Schedule(MilliSeconds(100), &HypatiaScheduledUpdate);
    }
    Simulator::Schedule(Seconds(g_tickIntervalS), &OnTick);

    Simulator::Stop(simTimeNs3);
    Simulator::Run();

    // ------------------------------------------------------------------------
    // Résumé final console
    // ------------------------------------------------------------------------
    g_monitor->CheckForLostPackets();
    auto stats = g_monitor->GetFlowStats();
    std::cout << "\n=== FINAL RESULTS (optionD NTN Multi-UE) ===\n";
    std::cout << "satAlt=" << g_satAltKm << "km  txPower=" << g_txPowerDbm
              << "dBm  numUEs=" << g_numUEs << "\n";

    // Per-flow output + aggregate
    double aggDl = 0, aggUl = 0;
    double sumDlX = 0, sumDlX2 = 0; int dlCount = 0;
    uint64_t totalDlTx = 0, totalDlRx = 0;
    for (const auto& kv : stats) {
        auto t = g_classifier->FindFlow(kv.first);
        FlowType ft = ClassifyFlow(t);
        std::string dir = (ft==FLOW_UL_DATA)?"UL":(ft==FLOW_DL_DATA)?"DL":"OTHER";
        double tput = kv.second.rxBytes * 8.0 / g_simTime / 1e6;
        std::cout << dir
                  << " " << t.sourceAddress << ":" << t.sourcePort
                  << "->" << t.destinationAddress << ":" << t.destinationPort
                  << "  TxPkt=" << kv.second.txPackets
                  << "  RxPkt=" << kv.second.rxPackets
                  << "  LostPkt=" << kv.second.lostPackets
                  << "  Tput=" << tput << " Mbps\n";
        if (ft == FLOW_DL_DATA) {
            aggDl += tput;
            sumDlX += tput; sumDlX2 += tput*tput; dlCount++;
            totalDlTx += kv.second.txPackets;
            totalDlRx += kv.second.rxPackets;
        }
        if (ft == FLOW_UL_DATA) aggUl += tput;
    }
    // Jain's Fairness Index
    double jain = (dlCount>0 && sumDlX2>0) ?
                  (sumDlX*sumDlX) / ((double)dlCount * sumDlX2) : 0.0;
    double pdrDl = (totalDlTx>0) ?
                   (double)totalDlRx / (double)totalDlTx : 0.0;
    double perUeDl = (dlCount>0) ? aggDl / dlCount : 0.0;
    std::cout << "\n--- AGGREGATE ---\n"
              << "  N_UE=" << g_numUEs
              << "  Aggregate_DL=" << aggDl << " Mbps"
              << "  Per-UE_DL=" << perUeDl << " Mbps"
              << "  Jain=" << jain
              << "  PDR_DL=" << pdrDl
              << "  Aggregate_UL=" << aggUl << " Mbps\n";

    g_csvFile.close();
    g_rttFile.close();
    Simulator::Destroy();
    return EXIT_SUCCESS;
}
