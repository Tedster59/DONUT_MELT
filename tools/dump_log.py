"""Download the flash run log from the bot over USB serial.

Usage (run from the repo root; the port is found automatically if omitted):
    python tools/dump_log.py                      # saves donut_log_<timestamp>.csv
    python tools/dump_log.py COM3 -o run1.csv
    python tools/dump_log.py --plot               # also plots RSSI, RPM and traction control (needs matplotlib)
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
        ncols = None    # from the header line
        garbled = 0     # lines with bytes lost in the USB transfer
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
            if line.startswith("kind,"):
                ncols = len(line.split(","))
                lines.append(line)
                deadline = time.time() + 5
            elif line and line[0] in RECORD_KINDS + "".join(EVENT_KINDS):
                # USB serial occasionally drops bytes; a damaged line has the wrong column count,
                # a line break inside it, or no timestamp. Keep only intact lines.
                fields = line.split(",")
                if ("\r" in line or (ncols is not None and len(fields) != ncols)
                        or len(fields) < 2 or not fields[1].isdigit()):
                    garbled += 1
                else:
                    lines.append(line)
                deadline = time.time() + 5  # keep going while data flows
            elif line:
                garbled += 1
        else:
            sys.exit("Timed out waiting for the dump (is the killswitch on?).")

    with open(out_path, "w", newline="") as f:
        f.write("\n".join(lines) + "\n")
    print(f"Wrote {len(lines) - 1} records to {out_path}")
    if garbled:
        print(f"  skipped {garbled} line(s) damaged in the USB transfer")


FAILSAFE_REASONS = {1: "link LQ/RSSI", 2: "RC timeout"}
CUT_REASONS = {1: "failsafe", 2: "killswitch"}
EVENT_KINDS = ("F", "f", "W", "L", "l", "K", "k", "G", "B")
BOOT_REASONS = {1: "power-on", 2: "BROWNOUT", 4: "SUPPLY GLITCH", 8: "RUN pin", 16: "WATCHDOG TIMEOUT",
                32: "reboot after flashing (bootrom/picotool)"}
RECORD_KINDS = "RAPDCMTUVSHJ"
HD_STATES = {0: "idle", 1: "acquiring", 2: "locked", 3: "holding"}
TC_STATES = {0: "off", 1: "uncalibrated", 2: "no ESC telemetry", 3: "watching", 4: "limiting"}


def read_traction(path):
    """Traction control records: motor rpm (M), state + fast accel rpm (T), cap + reference (U),
    slip + wheel rpm in bot-equivalent rpm (V), slip target when it changes (S, from the S1 slider).
    Logs from before traction control have none."""
    tc = {"m_t": [], "m_rpm": [], "t_t": [], "state": [], "accel": [], "u_t": [], "cap": [], "ref": [],
          "v_t": [], "slip": [], "wheel": [], "s_t": [], "target": []}
    pending = {}
    with open(path) as f:
        for row in csv.DictReader(f):
            kind = row["kind"]
            if kind not in "MTUVS":
                continue
            t = int(row["t_us"]) / 1e6
            if kind == "M":
                # motors 1 and 2 are logged back to back; keep the faster one (the one that slips)
                pending[int(row["motor"])] = int(row["motor_rpm"])
                if len(pending) == 2:
                    tc["m_t"].append(t)
                    tc["m_rpm"].append(max(pending.values()))
                    pending = {}
            elif kind == "T":
                tc["t_t"].append(t)
                tc["state"].append(int(row["tc_state"]))
                tc["accel"].append(int(row["tc_accel_rpm"]))
            elif kind == "U":
                tc["u_t"].append(t)
                tc["cap"].append(float(row["tc_ceiling"]))
                tc["ref"].append(int(row["tc_ref_rpm"]))
            elif kind == "S":
                tc["s_t"].append(t)
                tc["target"].append(float(row["tc_target"]))
            else:
                tc["v_t"].append(t)
                tc["slip"].append(float(row["tc_slip"]))
                tc["wheel"].append(int(row["tc_wheel_rpm"]))
    return tc


def read_heading(path):
    """RSSI heading tracker records: state + tracker rpm (H), match quality + phase error (J)."""
    hd = {"h_t": [], "state": [], "rpm": [], "j_t": [], "quality": [], "err": []}
    with open(path) as f:
        for row in csv.DictReader(f):
            if row["kind"] == "H":
                hd["h_t"].append(int(row["t_us"]) / 1e6)
                hd["state"].append(int(row["hd_state"]))
                hd["rpm"].append(float(row["hd_rpm"]))
            elif row["kind"] == "J":
                hd["j_t"].append(int(row["t_us"]) / 1e6)
                hd["quality"].append(float(row["hd_quality"]))
                hd["err"].append(float(row["hd_err_deg"]))
    return hd


def summarize_heading(hd):
    if not hd["h_t"]:
        return
    n = len(hd["state"])
    locked = [i for i, s in enumerate(hd["state"]) if s == 2]
    print(f"RSSI heading: {n} updates, locked {100 * len(locked) / n:.0f}%, holding "
          f"{100 * hd['state'].count(3) / n:.0f}%, acquiring {100 * hd['state'].count(1) / n:.0f}%, "
          f"idle {100 * hd['state'].count(0) / n:.0f}%")
    errs = [hd["err"][i] for i in locked if i < len(hd["err"]) and hd["quality"][i] >= 0.5]
    if len(errs) > 5:
        mean = sum(errs) / len(errs)
        std = (sum((e - mean) ** 2 for e in errs) / len(errs)) ** 0.5
        print(f"  while locked: phase error per update std {std:.1f} deg, "
              f"median match quality {sorted(hd['quality'][i] for i in locked)[len(locked) // 2]:.2f}")


def suggest_calibration(tc, min_accel_rpm=800):
    """TC_MOTOR_RPM_PER_BOT_RPM from motor rpm / accel rpm. Slip only ever makes the wheels faster
    than the bot while driving, so a low percentile of the ratio is the no-slip value."""
    ratios = [m / a for m, a in zip(tc["m_rpm"], tc["accel"]) if a >= min_accel_rpm and m > 0]
    if len(ratios) < 50:
        print(f"Traction calibration: not enough samples above {min_accel_rpm} accel rpm "
              f"({len(ratios)}); spin steadily for a few seconds.")
        return None
    ratios.sort()
    pct = lambda p: ratios[int(p / 100 * (len(ratios) - 1))]
    print(f"Traction calibration from {len(ratios)} samples: motor rpm / bot rpm 10th pct {pct(10):.3f}, "
          f"median {pct(50):.3f}, 90th pct {pct(90):.3f}")
    print(f"  suggested: #define TC_MOTOR_RPM_PER_BOT_RPM {pct(10):.3f}"
          "  (from the gentlest, steadiest spins; check the ratio plot is flat there)")
    return pct(10)


def read_events(path):
    """Event rows (failsafe, watchdog, live, killswitch, CH5 glitch) as (t_s, kind, row)."""
    with open(path) as f:
        return [(int(r["t_us"]) / 1e6, r["kind"], r) for r in csv.DictReader(f) if r["kind"] in EVENT_KINDS]


def describe_event(kind, row):
    ch5 = row.get("ch5") or "?"  # logs from before v3 have no ch5 column
    bits = int(row["fs_reasons"] or 0)
    if kind == "B":
        why = " + ".join(name for bit, name in BOOT_REASONS.items() if bits & bit) or "unknown"
        return f"board booted before this run: {why}"
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

    tc = read_traction(path)
    has_tc = bool(tc["m_t"])
    hd = read_heading(path)
    summarize_heading(hd)
    fig, axes = plt.subplots(5 if has_tc else 3, 1, sharex=True, figsize=(12, 12 if has_tc else 8))
    axes[0].plot(rssi_t, rssi, lw=0.5)
    axes[0].set_ylabel("RSSI (dBm)")
    axes[1].plot(a_t, accel_rpm, label="accel raw", alpha=0.5)
    if hd["h_t"]:
        locked_t = [t for t, st_ in zip(hd["h_t"], hd["state"]) if st_ == 2]
        locked_rpm = [r for r, st_ in zip(hd["rpm"], hd["state"]) if st_ == 2]
        axes[1].plot(locked_t, locked_rpm, ".", ms=2, color="black", label="RSSI heading tracker (locked)")
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
    axes[-1].set_xlabel("time (s)")
    axes[2].legend()

    if has_tc:
        # wheel speed vs bot speed: wheel_rpm is in bot-equivalent rpm once calibrated; before
        # that, the motor/accel ratio is plotted so the no-slip value can be read off
        ax = axes[3]
        ax.plot(tc["t_t"], tc["accel"], label="bot (fast accel)")
        ax.plot(tc["u_t"], tc["ref"], label="TC reference", lw=0.8, ls="--")
        if tc["v_t"]:
            ax.plot(tc["v_t"], tc["wheel"], label="wheels (bot-equivalent)")
            ax.set_ylabel("RPM")
        else:
            ratio_t = [t for t, a in zip(tc["m_t"], tc["accel"]) if a >= 800]
            ratio = [m / a for m, a in zip(tc["m_rpm"], tc["accel"]) if a >= 800]
            ax2 = ax.twinx()
            ax2.plot(ratio_t, ratio, color="tab:red", marker=".", ls="none", ms=2, label="motor/bot ratio")
            ax2.set_ylabel("motor rpm / bot rpm")
            ax2.legend(loc="upper right")
            ax.set_ylabel("bot RPM")
        ax.legend(loc="upper left")
        ax = axes[4]
        ax.plot(tc["u_t"], tc["cap"], label="throttle cap")
        if tc["v_t"]:
            ax.plot(tc["v_t"], tc["slip"], label="slip")
        if tc["s_t"]:
            # the target is only logged when it changes: hold it to the end of the log
            end = max(tc["t_t"][-1] if tc["t_t"] else 0, tc["s_t"][-1])
            ax.step(tc["s_t"] + [end], tc["target"] + tc["target"][-1:], where="post", ls="--",
                    color="black", lw=1, label="slip target")
        ax.step(tc["t_t"], [s / 4 for s in tc["state"]], where="post", lw=0.8,
                label="TC state / 4 (" + ", ".join(f"{k}={v}" for k, v in TC_STATES.items()) + ")")
        ax.set_ylim(-0.2, 1.6)
        ax.legend(loc="upper right", fontsize=8)
        axes[-1].set_xlabel("time (s)")

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

    # RSSI heading tracker state as a strip along the top of each panel (each H record holds until the
    # next one, or one update period at the end / across gaps)
    hd_colors = {1: "gray", 2: "tab:green", 3: "gold"}
    if hd["h_t"]:
        runs = []  # (start, end, state)
        for i, (t, s) in enumerate(zip(hd["h_t"], hd["state"])):
            end = hd["h_t"][i + 1] if i + 1 < len(hd["h_t"]) else t + 0.05
            end = min(end, t + 0.1)  # don't bridge gaps without updates (tracker idle)
            if runs and runs[-1][2] == s and abs(runs[-1][1] - t) < 1e-9:
                runs[-1] = (runs[-1][0], end, s)
            else:
                runs.append((t, end, s))
        for ax in axes:
            for start, end, s in runs:
                if s in hd_colors:
                    ax.axvspan(start, end, ymin=0.95, ymax=1.0, color=hd_colors[s], alpha=0.9, lw=0)
    from matplotlib.patches import Patch
    axes[0].legend(handles=[
        Patch(color="red", alpha=0.15, label="RSSI rpm unlocked (shading)"),
        Patch(color="tab:green", label="heading tracker locked (top strip)"),
        Patch(color="gold", label="heading tracker holding"),
        Patch(color="gray", label="heading tracker acquiring (no strip = idle)"),
    ], loc="lower left", fontsize=8)

    # failsafe start (red) / end (green), watchdog reboots (black), cuts (orange) / back live (blue),
    # killswitch (purple), filtered CH5 glitches (magenta, dotted), boot reason (gray)
    colors = {"F": "red", "f": "green", "W": "black", "L": "orange", "l": "blue", "K": "purple", "k": "purple",
              "G": "magenta", "B": "gray"}
    for t, kind, row in read_events(path):
        for ax in axes:
            ax.axvline(t, color=colors[kind], ls=":" if kind == "G" else "--", lw=1)
        if kind in ("F", "L", "G"):
            axes[0].annotate(describe_event(kind, row).split(" (")[0], (t, 1), xycoords=("data", "axes fraction"),
                             rotation=90, va="top", ha="right", fontsize=8, color=colors[kind])
    # time axis = the logged records only: the RAM log keeps the last ~28 s of a longer run, and the
    # boot event at t = 0 would otherwise stretch the axis back to boot
    data_t = [ts[i] for ts in (rssi_t, a_t, p_t, d_t, c_t) if ts for i in (0, -1)]
    if data_t:
        pad = 0.01 * (max(data_t) - min(data_t))
        for ax in axes:
            ax.set_xlim(min(data_t) - pad, max(data_t) + pad)
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
        suggest_calibration(read_traction(args.plot_only))
        plot(args.plot_only)
        return
    port = args.port or find_pico_port()
    print(f"Using {port}")

    out = args.out or datetime.datetime.now().strftime("donut_log_%Y%m%d_%H%M%S.csv")
    download(port, out)
    print_events(out)
    suggest_calibration(read_traction(out))
    if args.plot:
        plot(out)


if __name__ == "__main__":
    main()
