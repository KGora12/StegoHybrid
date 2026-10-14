# StegoHybrid

StegoHybrid provides the code needed to run and evaluate a fully hybrid steganographic channel that combines the PHY and MAC layers of IEEE 802.11ax, implemented in ns-3 (version 3.44).

The hybrid channel consists of two covert sub-channels:

- **PHY sub-channel (Next-generation dirty constellation).** Secret bits are embedded by displacing selected QAM-16 symbols into an elliptical ring around their nominal constellation points. The receiver recovers them from symbols that fall outside an inner decision ellipse.
- **MAC sub-channel (TXOP).** Secret bits are encoded in the size of frames sent within EDCA transmission opportunities, alternating between the AC_VO and AC_VI access categories.

A controller decides in each step whether only the PHY sub-channel is used or whether a TXOP symbol is sent as well.

## Repository structure

```
.
├── ns3/
│   ├── scratch/
│   │   ├── channel_s1.cc        # Scenario 1: PHY sub-channel (QAM-16 stego, SNR sweep)
│   │   └── channel_s2_4.cc      # Scenarios 2–4: hybrid PHY + MAC channel
│   └── src/wifi/model/          # Reference copies of the ns-3.44 files the channel relies on
├── data/
│   ├── scenario1/
│   │   ├── SNR/data_s1.csv                      # PER, BER and hidden BER vs. SNR
│   │   └── constellation/all_constellation/...  # sent/received constellation points per SNR
│   ├── scenario2/data_s2/
│   ├── scenario3/data_s3/
│   └── scenario4/data_s4/                       # results for each hybrid mode
├── LICENSE
└── README.md
```

The files in `ns3/src/wifi/model/` are **unmodified** copies of the ns-3.44 sources (EDCA/TXOP and YANS PHY/channel). They are included for reference only and do not need to be copied into your ns-3 tree. The covert channel is implemented entirely in the programs in `ns3/scratch/`, which configure the standard ns-3 Wi-Fi module through its public API.

## Requirements

- ns-3.44
- A C++17 compiler (GCC or Clang) and CMake, as required by ns-3
- Python 3 (optional, for processing the CSV results)

## Installation

1. Download and build ns-3.44 following the [official instructions](https://www.nsnam.org/docs/installation/html/).
2. Copy the simulation programs into the `scratch/` directory of your ns-3 installation:
   ```bash
   cp ns3/scratch/*.cc /path/to/ns-3.44/scratch/
   ```
3. Build ns-3:
   ```bash
   cd /path/to/ns-3.44
   ./ns3 build
   ```

## Usage

### Scenario 1: PHY sub-channel

Simulates QAM-16 steganography over 802.11ax (HE-MCS3, 20 MHz) for a given SNR and set of ellipse thresholds.

```bash
./ns3 run "channel_s1 --snrDb=24"
```

Main parameters:

| Parameter | Description | Default |
|-----------|-------------|---------|
| `--snrDb` | SNR in dB used to generate EVM-based noise | 24 |
| `--e1` | Inner ellipse threshold (decoder) | 1/300 |
| `--e2` | Lower bound of the embedding ring | 1/70 |
| `--e3` | Upper bound of the embedding ring | 1/40 |
| `--sim` | Simulation time in seconds | 300 |
| `--data` | UDP payload size in bytes | 2000 |

Each run appends one row (`snr_db, qam_per, qam_ber, hidden_ber, e1, e2, e3`) to `data_s1.csv` and writes the constellation of the first packet to `all_constellation/<e1>_<e2>_<e3>/all_constellation_<snr>.csv`.

An SNR sweep can be run with a simple loop:

```bash
for snr in $(seq 0 30); do
  ./ns3 run "channel_s1 --snrDb=$snr"
done
```

### Scenarios 2–4: hybrid channel

Simulates the hybrid PHY + MAC channel with optional background stations.

```bash
# Adaptive TXOP Mode: a TXOP symbol is sent in a random 20% of the steps
./ns3 run "channel_s2_4 --pOne=0.2"

# Only Physical Layer Mode: QAM steganography only
./ns3 run "channel_s2_4 --random=0 --combo=0"

# TXOP Full-Usage Mode: a TXOP symbol is sent in every step
./ns3 run "channel_s2_4 --random=0 --combo=1"
```

Main parameters:

| Parameter | Description | Default |
|-----------|-------------|---------|
| `--random` | Random hybrid mode (1) or fixed cyclic pattern (0) | 1 |
| `--pOne` | Random mode: probability of a QAM + TXOP step | 0.2 |
| `--combo` | Cyclic pattern used when `random=0` (`0` = QAM, `1` = QAM + TXOP) | `00000` |
| `--snrDb` | QAM Es/N0 in dB | 26 |
| `--stations` | Number of background stations | 0 |
| `--bgRate` | Traffic rate per background station | 25Mbps |
| `--legBgRate` | Target QAM rate (`0Mbps` disables QAM) | 5Mbps |
| `--sim` | Simulation time in seconds | 300 |
| `--csv` | Output CSV file | `hybrid_cc_results.csv` |

Run `./ns3 run "channel_s2_4 --help"` for the full list of parameters.

Each run prints a report and appends one row to the CSV file with TXOP goodput, delay and jitter, QAM PER/BER and goodput, hidden BER and goodput, and IP-level metrics from FlowMonitor.

## Results

The `data/` directory contains the results used in the paper:

- `scenario1/` contains the PHY sub-channel results: error rates as a function of SNR and the sent/received constellation points.
- `scenario2/` – `scenario4/` contain the hybrid channel results for the three operating modes: *Only Physical Layer Mode*, *Adaptive TXOP Mode* and *TXOP Full-Usage Mode*.

## Citation

If you use this code in your research, please cite:

```bibtex
@article{stegohybrid,
  author  = {TODO},
  title   = {TODO},
  journal = {Computer Communications},
  year    = {TODO},
  doi     = {TODO}
}
```

## License

This project is licensed under the GNU General Public License v2.0 (GPL-2.0-only), the same license as ns-3. See [LICENSE](LICENSE) for details.
