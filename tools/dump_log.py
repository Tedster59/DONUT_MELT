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
            if line and (line[0] in "RAPDCFfW" or line.startswith("kind,")):
                lines.append(line)
                deadline = time.time() + 5  # keep going while data flows
        else:
            sys.exit("Timed out waiting for the dump (is the killswitch on?).")

    with open(out_path, "w", newline="") as f:
        f.write("\n".join(lines) + "\n")
    print(f"Wrote {len(lines) - 1} records to {out_path}")


FAILSAFE_REASONS = {1: "link LQ/RSSI", 2: "RC timeout"}
CUT_REASONS = {1: "failsafe", 2: "killswitch"}
EVENT_KINDS = ("F", "f", "W", "L", "l", "K", "k", "G")


def read_events(path):
    """Event rows (failsafe, watchdog, live, killswitch, CH5 glitch) as (t_s, kind, row)."""
    with open(path) as f:
        return [(int(r["t_us"]) / 1e6, r["kind"], r) for r in csv.DictReader(f) if r["kind"] in EVENT_KINDS]


def describe_event(kind, row):
    ch5 = row.get("ch5") or "?"  # logs from before v3 have no ch5 column
    bits = int(row["fs_reasons"] or 0)
    if kind == "W":
        return "run started after a WATCHDOG REBOOT (the previous run's RAM log was lost)"
    if kind == "f":
        return f"failsafe cleared after {row['fs_ms']} ms"
    if kind == "F":
        why = " + ".join(name for bit, name in FAILSAFE_REASONS.items() if bits & bit) or "unknown"
        return f"FAILSAFE: {why} (last RC frame {row['fs_ms']} ms ago, rssi -{row['rssi']} dBm, lq {row['lq']}%)"
    if kind == "L":
        why = " + ".join(name for bit, name in CUT_REASONS.items() if bits & bit) or "unknown"
        return f"CUT (not live): {why} (CH5 {ch5}, rssi -{row['rssi']} dBm, lq {row['lq']}%)"
    if kind == "l":
        return f"live again after {row['fs_ms']} ms"
    if kind == "K":
        return f"killswitch ON (CH5 {ch5}, read killed for {row['fs_ms']} ms first)"
    if kind == "k":
        return f"killswitch off (CH5 {ch5})"
    # G
    which = "KILLED while armed" if bits == 1 else "armed while killed"
    return f"CH5 GLITCH filtered: read {which} for {row['fs_ms']} ms (CH5 {ch5}, rssi -{row['rssi']} dBm, lq {row['lq']}%)"


def print_events(path):
    events = read_events(path)
    if not events:
        print("No events.")
    for t, kind, row in events:
        print(f"  {t:8.3f} s  {describe_event(kind, row)}")


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

    # failsafe start (red) / end (green), watchdog reboots (black), cuts (orange) / back live (blue),
    # killswitch (purple), filtered CH5 glitches (magenta, dotted)
    colors = {"F": "red", "f": "green", "W": "black", "L": "orange", "l": "blue", "K": "purple", "k": "purple",
              "G": "magenta"}
    for t, kind, row in read_events(path):
        for ax in axes:
            ax.axvline(t, color=colors[kind], ls=":" if kind == "G" else "--", lw=1)
        if kind in ("F", "L", "G"):
            axes[0].annotate(describe_event(kind, row).split(" (")[0], (t, 1), xycoords=("data", "axes fraction"),
                             rotation=90, va="top", ha="right", fontsize=8, color=colors[kind])
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
        print_events(args.plot_only)
        plot(args.plot_only)
        return
    port = args.port or find_pico_port()
    print(f"Using {port}")

    out = args.out or datetime.datetime.now().strftime("donut_log_%Y%m%d_%H%M%S.csv")
    download(port, out)
    print_events(out)
    if args.plot:
        plot(out)


if __name__ == "__main__":
    main()
