#!/usr/bin/env python3
"""
run_matrix.py : run mqtt_stress.py for several QoS levels and repeats under ONE test condition.

You set the condition (broker CPU limit, packet loss, ...) by hand, then run this once per condition:
    python run_matrix.py --label base                      # unrestricted broker, no loss
    python run_matrix.py --label cpu05                     # after starting the broker with --cpus=0.5
    python run_matrix.py --label loss1                     # after enabling 1 % packet loss (netem / Clumsy)

Output files are named <label>_q<qos>_r<rep>_latency.csv / _summary.csv.
Time estimate: about 2.5 minutes per (qos, repeat) with the default 6 load levels.
"""
import argparse, subprocess, sys, time

ap = argparse.ArgumentParser()
ap.add_argument("--label", required=True, help="name of the test condition, e.g. base, cpu05, loss1")
ap.add_argument("--qos", default="0,1")
ap.add_argument("--reps", type=int, default=3)
ap.add_argument("--cars", default="0,100,250,500,1000,2000")
ap.add_argument("--duration", default="20")
ap.add_argument("--pubs", default="4")
ap.add_argument("--host", default="localhost")
ap.add_argument("--port", default="1883")
ap.add_argument("--size", default="300")
ap.add_argument("--rate", default="10")
a = ap.parse_args()

for q in a.qos.split(","):
    for r in range(1, a.reps + 1):
        tag = "%s_q%s_r%d" % (a.label, q, r)
        print("\n===== %s =====" % tag, flush=True)
        cmd = [sys.executable, "mqtt_stress.py", "--cars", a.cars, "--qos", q, "--tag", tag,
               "--duration", a.duration, "--pubs", a.pubs, "--host", a.host, "--port", a.port,
               "--size", a.size, "--rate", a.rate]
        subprocess.run(cmd, check=True)
        time.sleep(3)          # let the broker settle between runs
print("\nAll runs finished for condition '%s'." % a.label)
