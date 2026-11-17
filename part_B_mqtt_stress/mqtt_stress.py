#!/usr/bin/env python3
"""
mqtt_stress.py : load a Mosquitto broker with N simulated cars and measure probe latency/loss.

Why Python and not MATLAB for this part?  MATLAB's mqttclient cannot generate 10k+ msg/s, and
MATLAB callback scheduling would pollute the latency numbers.  MATLAB is used afterwards
(v2x_safety_vs_load.m) to turn the measured latency/loss into physical safety margins.

Install:  pip install paho-mqtt
Run:      python mqtt_stress.py --cars 0,100,250,500,1000,2000 --qos 0 --tag q0
          python mqtt_stress.py --cars 0,100,250,500,1000,2000 --qos 1 --tag q1

Design
  * Each level N spawns up to --pubs publisher PROCESSES (bypasses the Python GIL). Together they
    emulate N cars, each publishing --size bytes at --rate Hz to its own topic v2x/load/<id>/telemetry.
  * A SINK process subscribes to v2x/load/# (the RSU edge application). Without a subscriber the broker
    does no fan-out work, so this matters for realism. Disable with --no-sink.
  * A PROBE client (the "lead -> follower" beacon) publishes timestamped messages at --probe-rate Hz to
    v2x/probe/echo and receives them back; latency = time from publish to delivery (same clock).
  * Outputs: <tag>_latency.csv (cars,qos,seq,latency_ms) and <tag>_summary.csv.
Limits: N cars are emulated by --pubs connections (not N TCP connections). The script reports the
ACHIEVED message rate; if it is below the offered rate, the generator (not the broker) saturated.
"""
import argparse, csv, multiprocessing as mp, os, threading, time
import paho.mqtt.client as mqtt


def make_client(cid):
    try:
        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=cid)   # paho-mqtt 2.x
    except AttributeError:
        c = mqtt.Client(client_id=cid)                                      # paho-mqtt 1.x
    c.max_inflight_messages_set(5000)
    c.max_queued_messages_set(0)
    return c


def connect(c, host, port):
    ev = threading.Event()

    def on_connect(client, userdata, flags, rc, *args):
        ev.set()

    c.on_connect = on_connect
    c.connect(host, port, keepalive=60)
    c.loop_start()
    if not ev.wait(5):
        raise RuntimeError("Cannot reach broker at %s:%d" % (host, port))


def publisher(host, port, wid, n_cars, rate, qos, size, run_s, sent_val):
    c = make_client("load_%d_%d" % (wid, os.getpid()))
    connect(c, host, port)
    payload = b"x" * size
    topics = ["v2x/load/c%d_%d/telemetry" % (wid, i) for i in range(n_cars)]
    groups = [topics[g::10] for g in range(10)]        # spread cars over 10 sub-ticks (desynchronised)
    tick = 1.0 / rate / 10.0
    t_next = time.perf_counter()
    t_end = t_next + run_s
    sent, g = 0, 0
    while t_next < t_end:
        for tp in groups[g]:
            c.publish(tp, payload, qos)
            sent += 1
        g = (g + 1) % 10
        t_next += tick
        d = t_next - time.perf_counter()
        if d > 0:
            time.sleep(d)
    with sent_val.get_lock():
        sent_val.value += sent
    time.sleep(0.5)
    c.loop_stop()
    c.disconnect()


def sink(host, port, run_s, count_val):
    c = make_client("sink_%d" % os.getpid())
    n = [0]

    def on_msg(cl, ud, m):
        n[0] += 1

    c.on_message = on_msg
    connect(c, host, port)
    c.subscribe("v2x/load/#", 0)
    time.sleep(run_s)
    count_val.value = n[0]
    c.loop_stop()
    c.disconnect()


def run_level(a, n_cars):
    run_s = a.warmup + a.duration + 1.5
    procs, sent, got = [], mp.Value("l", 0), mp.Value("l", 0)
    if n_cars > 0:
        if not a.no_sink:
            p = mp.Process(target=sink, args=(a.host, a.port, run_s + 1.0, got)); p.start(); procs.append(p)
            time.sleep(0.7)
        k = min(a.pubs, n_cars)
        base, rem = divmod(n_cars, k)
        for w in range(k):
            n = base + (1 if w < rem else 0)
            p = mp.Process(target=publisher,
                           args=(a.host, a.port, w, n, a.rate, a.qos, a.size, run_s, sent))
            p.start(); procs.append(p)

    rx = {}
    probe = make_client("probe_%d" % os.getpid())

    def on_msg(cl, ud, m):
        now = time.perf_counter_ns()
        s, t0 = m.payload.split(b",")
        rx[int(s)] = (now - int(t0)) / 1e6

    probe.on_message = on_msg
    connect(probe, a.host, a.port)
    probe.subscribe("v2x/probe/echo", a.qos)
    time.sleep(0.5 + a.warmup)

    n_probe, period = int(a.duration * a.probe_rate), 1.0 / a.probe_rate
    t0 = time.perf_counter()
    for i in range(1, n_probe + 1):
        probe.publish("v2x/probe/echo", ("%d,%d" % (i, time.perf_counter_ns())).encode(), a.qos)
        d = t0 + i * period - time.perf_counter()
        if d > 0:
            time.sleep(d)
    time.sleep(1.0)
    probe.loop_stop(); probe.disconnect()
    for p in procs:
        p.join()
    return rx, n_probe, n_cars * a.rate, sent.value / run_s, got.value / run_s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--cars", default="0,100,250,500,1000,2000")
    ap.add_argument("--rate", type=float, default=10.0, help="msgs/s per simulated car")
    ap.add_argument("--qos", type=int, default=0, choices=[0, 1, 2])
    ap.add_argument("--size", type=int, default=300, help="payload bytes")
    ap.add_argument("--duration", type=float, default=20.0, help="probe seconds per level")
    ap.add_argument("--warmup", type=float, default=2.0)
    ap.add_argument("--pubs", type=int, default=4, help="publisher processes")
    ap.add_argument("--probe-rate", dest="probe_rate", type=float, default=10.0)
    ap.add_argument("--no-sink", dest="no_sink", action="store_true")
    ap.add_argument("--tag", default="stress")
    a = ap.parse_args()

    with open(a.tag + "_latency.csv", "w", newline="") as fl, open(a.tag + "_summary.csv", "w", newline="") as fs:
        wl, ws = csv.writer(fl), csv.writer(fs)
        wl.writerow(["cars", "qos", "seq", "latency_ms"])
        ws.writerow(["cars", "qos", "offered_msgs_s", "achieved_msgs_s", "sink_msgs_s", "probes_sent", "probes_recv"])
        for n in [int(x) for x in a.cars.split(",")]:
            print("Level: %d cars (offered %.0f msg/s) ..." % (n, n * a.rate), flush=True)
            rx, n_probe, off, ach, snk = run_level(a, n)
            for s in sorted(rx):
                wl.writerow([n, a.qos, s, "%.4f" % rx[s]])
            ws.writerow([n, a.qos, "%.0f" % off, "%.0f" % ach, "%.0f" % snk, n_probe, len(rx)])
            fl.flush(); fs.flush()
            med = sorted(rx.values())[len(rx) // 2] if rx else float("nan")
            flag = ""
            if ach < 0.95 * off:
                flag += "  <-- GENERATOR SATURATED"
            if n > 0 and not a.no_sink and ach > 0 and snk < 0.95 * ach:
                flag += "  <-- RSU SINK RECEIVED ONLY %.0f%% OF PUBLISHED LOAD (backlog/overload)" % (100.0 * snk / ach)
            print("   probes %d/%d received | median %.2f ms | achieved %.0f msg/s | sink %.0f msg/s%s"
                  % (len(rx), n_probe, med, ach, snk, flag))
    print("Wrote %s_latency.csv and %s_summary.csv" % (a.tag, a.tag))


if __name__ == "__main__":
    mp.freeze_support()
    main()
