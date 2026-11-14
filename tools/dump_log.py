"""Download the flash run log from the bot over USB serial.

Usage (run from the repo root; the port is found automatically if omitted):
    python tools/dump_log.py                      # saves donut_log_<timestamp>.csv
    python tools/dump_log.py COM3 -o run1.csv
    python tools/dump_log.py --plot               # also plots RSSI and RPM (needs matplotlib)
    python tools/dump_log.py --plot-only run1.csv # plot an existing file

The bot must be powered with the killswitch on (not live) so it answers serial commands.
Requires pyserial: python -m pip install pyserial
"""
import argparse
import csv
import datetime
import sys
import time


def find_pico_port():
    from serial.tools import list_ports

    picos = [p.device for p in list_ports.comports() if p.vid == 0x2E8A]  # Raspberry Pi USB vendor ID
    if len(picos) == 1:
        return picos[0]
    if not picos:
        sys.exit("No Pico serial port found. Is the bot plugged in over USB?")
    sys.exit(f"Several Pico ports found ({', '.join(picos)}); pass one explicitly.")


def download(port, out_path, timeout_s=60):
    import serial  # pyserial

    with serial.Serial(port, 115200, timeout=1) as ser:
        ser.reset_input_buffer()
        ser.write(b"d")
        lines = []
        started = False
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            if line.startswith("LOGGER no saved run"):
                sys.exit("Bot reports no saved run.")
            if line.startswith("# DONUT_LOG"):
                started = True
                print(line)
                continue
            if not started:
                continue  # skip LINK diagnostics or other output before the dump
            if line == "# END":
                break
            if line and (line[0] in "RAPDC" or line.startswith("kind,")):
                lines.append(line)
                deadline = time.time() + 5  # keep going while data flows
        else:
            sys.exit("Timed out waiting for the dump (is the killswitch on?).")

    with open(out_path, "w", newline="") as f:
        f.write("\n".join(lines) + "\n")
    print(f"Wrote {len(lines) - 1} records to {out_path}")


def plot(path):
    import matplotlib.pyplot as plt

    rssi_t, rssi = [], []
    a_t, accel_rpm, throttle = [], [], []
    p_t, rssi_rpm, conf = [], [], []
    d_t, state, drive_rpm = [], [], []
    c_t, corrected_rpm = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            t = int(row["t_us"]) / 1e6
            if row["kind"] == "R":
                rssi_t.append(t)
                rssi.append(-int(row["rssi"]))
            elif row["kind"] == "A":
                a_t.append(t)
                accel_rpm.append(int(row["accel_rpm"]))
                throttle.append(int(row["throttle_pct"]))
            elif row["kind"] == "P":
                p_t.append(t)
                rssi_rpm.append(int(row["rssi_rpm"]))
                conf.append(float(row["confidence"]))
            elif row["kind"] == "D":  # logs from before the D/C records simply have none
                d_t.append(t)
                state.append(int(row["rssi_state"]))
                drive_rpm.append(int(row["drive_rpm"]))
            elif row["kind"] == "C":
                c_t.append(t)
                corrected_rpm.append(int(row["accel_rpm_corrected"]))

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 8))
    axes[0].plot(rssi_t, rssi, lw=0.5)
    axes[0].set_ylabel("RSSI (dBm)")
    axes[1].plot(a_t, accel_rpm, label="accel raw", alpha=0.5)
    if c_t:
        axes[1].plot(c_t, corrected_rpm, label="accel corrected")
    axes[1].plot(p_t, rssi_rpm, label="rssi", marker=".", ls="none")
    if d_t:
        axes[1].plot(d_t, drive_rpm, label="used by drive", color="k", lw=0.8)
    axes[1].set_ylabel("RPM")
    axes[1].legend()
    axes[2].plot(a_t, throttle, label="throttle %")
    axes[2].plot(p_t, [c * 100 for c in conf], label="confidence x100")
    if d_t:
        # 0 = unlocked (on accel), 1 = locked in shadow mode, 2 = locked and driving
        axes[2].step(d_t, [s * 50 for s in state], where="post", label="rssi state x50")
    axes[2].set_xlabel("time (s)")
    axes[2].legend()

    # shade where RSSI was not locked, so fallbacks stand out
    for ax in axes:
        start = None
        for t, s in zip(d_t, state):
            if s == 0 and start is None:
                start = t
            elif s != 0 and start is not None:
                ax.axvspan(start, t, color="red", alpha=0.08, lw=0)
                start = None
        if start is not None:
            ax.axvspan(start, d_t[-1], color="red", alpha=0.08, lw=0)
    fig.tight_layout()
    plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", help="serial port, e.g. COM3 (default: auto-detect the Pico)")
    ap.add_argument("-o", "--out", help="output CSV path")
    ap.add_argument("--plot", action="store_true", help="plot after downloading")
    ap.add_argument("--plot-only", metavar="CSV", help="plot an existing CSV without downloading")
    args = ap.parse_args()

    if args.plot_only:
        plot(args.plot_only)
        return
    port = args.port or find_pico_port()
    print(f"Using {port}")

    out = args.out or datetime.datetime.now().strftime("donut_log_%Y%m%d_%H%M%S.csv")
    download(port, out)
    if args.plot:
        plot(out)


if __name__ == "__main__":
    main()
