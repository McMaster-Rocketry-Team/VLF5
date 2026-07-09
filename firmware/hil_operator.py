#!/usr/bin/env python3
"""HIL operator driver: walks the single-deploy operator sequence against a real
`rocket-cli control` session and asserts the flight plays out. Prints a PASS/FAIL
summary. All telemetry/commands are echoed with timestamps for the RTT/JSON logs.

Usage: hil_operator.py [--target 3000] [--no-selftest] [--freq HZ] [--key B64]
Env: ROCKET_CLI overrides the binary path.
"""
import argparse, json, os, subprocess, sys, threading, time

RC = os.environ.get(
    "ROCKET_CLI",
    "/home/pegasis/Projects/RocketryProjects/Rust_Monorepo/target/debug/rocket-cli",
)


def ts():
    return time.strftime("%H:%M:%S")


class Operator:
    def __init__(self, args):
        self.args = args
        cmd = [RC, "control"]
        if args.usb:
            cmd += ["--usb"]
        else:
            if args.freq:
                cmd += ["--frequency", str(args.freq)]
            if args.key:
                cmd += ["--vlp-key", args.key]
        self.p = subprocess.Popen(
            cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, text=True, bufsize=1,
        )
        self.lock = threading.Lock()
        self.link_up = False
        self.stages = []            # ordered unique flight_stage transitions
        self.acks = []              # {type: ack|nack|timeout, command, ...}
        self.self_test = None
        self.landed = False
        self.last_tel = None
        self.max_alt = 0.0
        self.saw_airbrakes_cmd = False
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        for line in self.p.stdout:
            line = line.strip()
            if not line:
                continue
            try:
                o = json.loads(line)
            except json.JSONDecodeError:
                print(f"{ts()} NONJSON {line}", flush=True)
                continue
            print(f"{ts()} RX {line}", flush=True)
            with self.lock:
                t = o.get("type")
                if t in ("low_power_telemetry", "telemetry", "landed_telemetry",
                         "gps_beacon", "self_test_result"):
                    self.link_up = True
                if t == "telemetry":
                    st = o.get("flight_stage")
                    if not self.stages or self.stages[-1] != st:
                        self.stages.append(st)
                    self.last_tel = o
                    self.max_alt = max(self.max_alt, o.get("max_altitude_agl", 0.0))
                    if (o.get("airbrakes_cmd_pct") or 0) > 0.01:
                        self.saw_airbrakes_cmd = True
                if t == "self_test_result":
                    self.self_test = o
                if t == "landed_telemetry":
                    self.landed = True
                if t in ("ack", "nack", "timeout", "sent"):
                    self.acks.append(o)

    def send(self, cmd):
        print(f"{ts()} TX {cmd}", flush=True)
        self.p.stdin.write(cmd + "\n")
        self.p.stdin.flush()

    def wait(self, cond, timeout, desc):
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.lock:
                if cond():
                    print(f"{ts()} OK  {desc}", flush=True)
                    return True
            time.sleep(0.2)
        print(f"{ts()} !!! TIMEOUT waiting for {desc}", flush=True)
        return False

    def acked(self, prefix):
        return any(a["type"] in ("ack", "sent") and a.get("command", "").startswith(prefix)
                   for a in self.acks)

    def close(self):
        try:
            self.send("quit")
            self.p.wait(timeout=5)
        except Exception:
            self.p.terminate()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", type=float, default=3000)
    ap.add_argument("--no-selftest", action="store_true")
    ap.add_argument("--usb", action="store_true", default=True, help="talk to VLF5 over USB (default)")
    ap.add_argument("--no-usb", dest="usb", action="store_false", help="use the GCM/LoRa path")
    ap.add_argument("--freq", type=int, default=None)
    ap.add_argument("--key", type=str, default=None)
    ap.add_argument("--land-timeout", type=int, default=300)
    ap.add_argument("--rtt-log", type=str, default=None,
                    help="path to the VLF5 RTT log, to confirm PyroDrogue/PyroMain fired")
    args = ap.parse_args()

    op = Operator(args)
    fails = []

    # 1. Link up (telemetry flowing)
    if not op.wait(lambda: op.link_up, 25, "link up (any telemetry)"):
        fails.append("no telemetry — radio absent / wrong frequency / wrong key")
        op.close(); return report(op, fails)

    # 2. Set target apogee
    op.send(f"target-apogee {args.target}")
    if not op.wait(lambda: op.acked("target-apogee"), 20, "target-apogee ack"):
        fails.append("target-apogee not acked")

    # 3. Self test (optional)
    if not args.no_selftest:
        op.send("mode self-test")
        # Self-test waits out CAN-node response timeouts; on the bench (AMP/Icarus/bulkheads
        # offline) that takes ~35s, so allow generous time.
        if op.wait(lambda: op.self_test is not None, 60, "self-test result"):
            st = op.self_test
            if not st.get("baro_ok"):
                fails.append("self-test baro_ok=false")
        else:
            fails.append("no self-test result")

    # 4. Arm
    op.send("arm")
    if not op.wait(lambda: op.acked("arm"), 20, "arm ack"):
        fails.append("arm not acked")

    # 5. Autonomous flight (pad hold ~15s, burn, coast, apogee, deploy). Coasting may be
    # skipped by the stage machine, and DrogueDeployed is transient (single-deploy delays=0
    # fires drogue->main in <1ms), so key on the telemetry-observable stages and confirm
    # the drogue fire from RTT (--rtt-log) separately.
    op.wait(lambda: "PoweredAscent" in op.stages, 90, "PoweredAscent (liftoff)")
    op.wait(lambda: "MainDeployed" in op.stages or "DrogueDeployed" in op.stages,
            150, "deploy (Drogue/Main)")

    # 6. Landing
    op.wait(lambda: op.landed or "Landed" in op.stages, args.land_timeout, "Landed")

    op.close()
    return report(op, fails)


def report(op, fails):
    print("\n==== SUMMARY ====", flush=True)
    print("stages:", " -> ".join(op.stages), flush=True)
    print("max_alt_agl:", round(op.max_alt, 1), "m", flush=True)
    print("saw airbrakes cmd>0:", op.saw_airbrakes_cmd, flush=True)
    print("acks:", op.acks, flush=True)

    for s in ["Armed", "PoweredAscent"]:
        if s not in op.stages:
            fails.append(f"missing stage {s}")
    # Deploy: at least one of Drogue/Main must show in telemetry (transient drogue may be
    # missed by 2s sampling); the RTT check below confirms both pyros actually fired.
    if not ("MainDeployed" in op.stages or "DrogueDeployed" in op.stages):
        fails.append("no deploy stage (Drogue/Main) in telemetry")
    if not (op.landed or "Landed" in op.stages):
        fails.append("never landed")

    if op.args.rtt_log:
        try:
            rtt = open(op.args.rtt_log).read()
        except OSError:
            rtt = ""
        for ev in ["FIRE PyroDrogue", "FIRE PyroMain"]:
            if ev not in rtt:
                fails.append(f"RTT missing '{ev}'")
            else:
                print(f"{ts()} OK  RTT saw {ev}", flush=True)

    if fails:
        print("RESULT: FAIL", flush=True)
        for f in fails:
            print("  -", f, flush=True)
        return 1
    print("RESULT: PASS", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
