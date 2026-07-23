"""
Larger-scale, multi-seed performance comparison: SCOSCA baseline vs. SCOSCA
tuned (CoSiCoSt-alignment fine-tuning), using production-like cycle bounds
(50-180s, matching demo_Bangalore_SCOSCA.py) instead of the shortened bounds
used by verify_scosca_tuning.py's mechanism-demonstration run.

This exists specifically to answer "is the improvement big enough to be
comparable to CoSiCoSt, or does it need more tuning" with normalized,
throughput-aware metrics (not raw averages, which are biased by how many
trips complete inside a fixed time window — see the writeup this script's
report references).

Usage:
    python verify_scosca_scaleup.py [--duration 1200] [--seeds 42,43,44] [--out DIR]
"""

import argparse
import io
import base64
import os
import statistics
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from verify_scosca_tuning import run_config, BASE_DIR  # reuse the simulation runner

NET_FILE = str(BASE_DIR / "Bangalore_Map" / "osm.net.xml.gz")

PROD_BASE_PARAMS = {
    "adaptation_cycle": 30,
    "adaptation_green": 10,
    "green_thresh": 2,
    "adaptation_offset": 1,
    "offset_thresh": 0.5,
    "min_cycle_length": 50,   # production bounds (demo_Bangalore_SCOSCA.py), not the
    "max_cycle_length": 180,  # shortened 15-50s used for mechanism verification
    "ds_upper_val": 0.925,
    "ds_lower_val": 0.875,
    "measurement_period": 4,
}
PROD_INITIAL_CYCLE_LENGTH = 120

TUNED_OVERRIDES = {
    "priority_stage_boost": 5.0,
    "priority_route_delay_weight": 0.05,
    "actuation_extension_sec": 5,
    "actuation_min_green_ratio": 0.5,
    "actuation_min_green_floor": 5,
    "actuation_gap_thresh": 3.0,
    "fallback_missing_cycles": 3,
    "fallback_min_green": 10,
    "fallback_max_green": 45,
}
BASELINE_OVERRIDES = {
    "priority_stage_boost": 0.0,
    "priority_route_delay_weight": 0.0,
    "actuation_extension_sec": 0,
    "actuation_min_green_ratio": 1.0,
    "actuation_gap_thresh": None,
    "fallback_missing_cycles": 10**9,
    "fallback_min_green": 10,
    "fallback_max_green": 45,
}

# CoSiCoSt's own published real-world ranges, from the papers in context/.
COSICOST_PROJECTED = {
    "speed_pct": (2, 12),        # Pune network eval: avg travel speed increase
    "delay_pct": (-30, -11),     # Pune network eval: avg delay reduction
    "volume_pct": (9.06, 9.06),  # Pune: traffic volume increase
}


def load_tripinfos(path):
    if not os.path.exists(path):
        return []
    root = ET.parse(path).getroot()
    trips = []
    for t in root.findall("tripinfo"):
        trips.append({
            "id": t.get("id"),
            "routeLength": float(t.get("routeLength")),
            "duration": float(t.get("duration")),
            "waitingTime": float(t.get("waitingTime")),
            "timeLoss": float(t.get("timeLoss")),
        })
    return trips


def summarize_run(tripinfo_path):
    trips = load_tripinfos(tripinfo_path)
    n = len(trips)
    if n == 0:
        return {"n_trips": 0, "speed": 0.0, "timeloss_per_m": 0.0, "waitingtime_per_m": 0.0}
    speed = statistics.mean(t["routeLength"] / t["duration"] for t in trips if t["duration"] > 0)
    timeloss_per_m = statistics.mean(t["timeLoss"] / max(1.0, t["routeLength"]) for t in trips)
    waitingtime_per_m = statistics.mean(t["waitingTime"] / max(1.0, t["routeLength"]) for t in trips)
    return {"n_trips": n, "speed": speed, "timeloss_per_m": timeloss_per_m, "waitingtime_per_m": waitingtime_per_m}


def pct_change(baseline, tuned):
    if baseline == 0:
        return float("nan")
    return (tuned - baseline) / abs(baseline) * 100.0


def fig_to_data_uri(fig):
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=110, bbox_inches="tight")
    plt.close(fig)
    return "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode("ascii")


def plot_metric_comparison(per_seed_results, metric, ylabel, title):
    baseline_vals = [r["baseline"][metric] for r in per_seed_results]
    tuned_vals = [r["tuned"][metric] for r in per_seed_results]
    fig, ax = plt.subplots(figsize=(5, 4))
    means = [statistics.mean(baseline_vals), statistics.mean(tuned_vals)]
    stds = [statistics.stdev(baseline_vals) if len(baseline_vals) > 1 else 0,
            statistics.stdev(tuned_vals) if len(tuned_vals) > 1 else 0]
    ax.bar(["baseline", "tuned"], means, yerr=stds, capsize=6, color=["#888", "#2b7"])
    ax.set_ylabel(ylabel)
    ax.set_title(title, fontsize=10)
    return fig_to_data_uri(fig)


def plot_throughput(per_seed_results):
    baseline_vals = [r["baseline"]["n_trips"] for r in per_seed_results]
    tuned_vals = [r["tuned"]["n_trips"] for r in per_seed_results]
    fig, ax = plt.subplots(figsize=(5, 4))
    x = range(len(per_seed_results))
    ax.plot(x, baseline_vals, "o--", label="baseline", color="#888")
    ax.plot(x, tuned_vals, "o-", label="tuned", color="#2b7")
    ax.set_xticks(list(x))
    ax.set_xticklabels([f"seed {r['seed']}" for r in per_seed_results])
    ax.set_ylabel("Trips completed in window")
    ax.set_title("Throughput per seed", fontsize=10)
    ax.legend(fontsize=8)
    return fig_to_data_uri(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=int, default=1200)
    parser.add_argument("--seeds", type=str, default="42,43,44")
    parser.add_argument("--out", type=str, default=str(Path(__file__).parent / "verify_scaleup_output"))
    args = parser.parse_args()

    seeds = [int(s) for s in args.seeds.split(",")]
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    baseline_params = {**PROD_BASE_PARAMS, **BASELINE_OVERRIDES}
    tuned_params = {**PROD_BASE_PARAMS, **TUNED_OVERRIDES}

    per_seed_results = []
    for seed in seeds:
        print(f"=== seed {seed}: baseline ===")
        _, baseline_stats_path, _ = run_config(
            f"baseline_seed{seed}", baseline_params, args.duration, out_dir,
            seed=seed, initial_cycle_length=PROD_INITIAL_CYCLE_LENGTH,
        )
        print(f"=== seed {seed}: tuned ===")
        _, tuned_stats_path, _ = run_config(
            f"tuned_seed{seed}", tuned_params, args.duration, out_dir,
            seed=seed, initial_cycle_length=PROD_INITIAL_CYCLE_LENGTH,
        )
        baseline_tripinfo = str(out_dir / f"baseline_seed{seed}_tripinfos.xml")
        tuned_tripinfo = str(out_dir / f"tuned_seed{seed}_tripinfos.xml")
        per_seed_results.append({
            "seed": seed,
            "baseline": summarize_run(baseline_tripinfo),
            "tuned": summarize_run(tuned_tripinfo),
        })
        print(f"  baseline: {per_seed_results[-1]['baseline']}")
        print(f"  tuned:    {per_seed_results[-1]['tuned']}")

    # Aggregate % change per seed, then mean/std across seeds (paired comparison
    # per seed controls for demand randomness; only the signal control differs).
    speed_pct = [pct_change(r["baseline"]["speed"], r["tuned"]["speed"]) for r in per_seed_results]
    timeloss_pct = [pct_change(r["baseline"]["timeloss_per_m"], r["tuned"]["timeloss_per_m"]) for r in per_seed_results]
    throughput_pct = [pct_change(r["baseline"]["n_trips"], r["tuned"]["n_trips"]) for r in per_seed_results]

    def fmt(vals):
        return f"{statistics.mean(vals):+.2f}% (std {statistics.stdev(vals):.2f})" if len(vals) > 1 else f"{vals[0]:+.2f}%"

    print("\n=== Aggregate paired % change (tuned vs baseline) across seeds ===")
    print("Speed:", fmt(speed_pct))
    print("Time-loss per meter:", fmt(timeloss_pct))
    print("Throughput (trips completed):", fmt(throughput_pct))

    img_speed = plot_metric_comparison(per_seed_results, "speed", "Avg speed (m/s)", "Average speed")
    img_timeloss = plot_metric_comparison(per_seed_results, "timeloss_per_m", "Time-loss per meter (s/m)", "Delay per unit distance")
    img_throughput = plot_throughput(per_seed_results)

    rows = "".join(
        f"<tr><td>seed {r['seed']}</td>"
        f"<td>{r['baseline']['n_trips']}</td><td>{r['tuned']['n_trips']}</td>"
        f"<td>{r['baseline']['speed']:.2f}</td><td>{r['tuned']['speed']:.2f}</td>"
        f"<td>{r['baseline']['timeloss_per_m']:.5f}</td><td>{r['tuned']['timeloss_per_m']:.5f}</td></tr>"
        for r in per_seed_results
    )

    def verdict_line(pct_vals, cosicost_range, label):
        mean_pct = statistics.mean(pct_vals)
        lo, hi = cosicost_range
        if lo <= mean_pct <= hi or (lo < 0 and mean_pct <= lo) or (lo > 0 and mean_pct >= hi):
            verdict = "within/beyond CoSiCoSt's reported range"
        elif abs(mean_pct) < abs(lo) * 0.25:
            verdict = "well below CoSiCoSt's range &mdash; more fine-tuning likely needed"
        else:
            verdict = "below CoSiCoSt's range but directionally correct &mdash; partial progress"
        return f"<tr><td>{label}</td><td>{mean_pct:+.2f}%</td><td>{lo:+.1f}% to {hi:+.1f}%</td><td>{verdict}</td></tr>"

    verdict_rows = (
        verdict_line(speed_pct, COSICOST_PROJECTED["speed_pct"], "Speed")
        + verdict_line([-x for x in timeloss_pct], (-COSICOST_PROJECTED["delay_pct"][1], -COSICOST_PROJECTED["delay_pct"][0]), "Delay reduction (timeLoss/m, sign-flipped)")
        + verdict_line(throughput_pct, COSICOST_PROJECTED["volume_pct"], "Throughput / volume")
    )

    html = f"""<h1>SCOSCA vs. CoSiCoSt (projected): multi-seed, production-cycle comparison</h1>
<p>Production cycle bounds (min/max {PROD_BASE_PARAMS['min_cycle_length']}-{PROD_BASE_PARAMS['max_cycle_length']}s,
initial {PROD_INITIAL_CYCLE_LENGTH}s &mdash; matching demo_Bangalore_SCOSCA.py), {args.duration}s per run,
{len(seeds)} seeds ({', '.join(str(s) for s in seeds)}), paired same-seed baseline vs. tuned comparison.
Metrics are throughput (trips completed in the fixed window) and per-distance normalized speed/time-loss,
not raw averages, to avoid the sample-composition bias raw duration/waitingTime averages are prone to
when two controllers complete different numbers of trips in the same window.</p>

<h2>Per-seed results</h2>
<table border="1" cellpadding="4" style="border-collapse:collapse">
<tr><th>Seed</th><th>Baseline trips</th><th>Tuned trips</th>
<th>Baseline speed (m/s)</th><th>Tuned speed (m/s)</th>
<th>Baseline timeLoss/m</th><th>Tuned timeLoss/m</th></tr>
{rows}
</table>

<h2>Aggregate % change (tuned vs. baseline) vs. CoSiCoSt's projected range</h2>
<table border="1" cellpadding="4" style="border-collapse:collapse">
<tr><th>Metric</th><th>Measured (this SCOSCA test)</th><th>CoSiCoSt projected (papers)</th><th>Verdict</th></tr>
{verdict_rows}
</table>

<h2>Plots</h2>
<img src="{img_speed}" style="max-width:48%">
<img src="{img_timeloss}" style="max-width:48%">
<img src="{img_throughput}" style="max-width:100%">

<p><i>CoSiCoSt's projected ranges are from real deployments (Pune network: 2-12% speed increase,
11-30% delay reduction, 9.06% traffic volume increase &mdash; from Adaptive_Signal_Control_Technology_State.pdf)
on real corridors over extended real-world periods. This test is {args.duration}s of synthetic OSM-derived
demand on a 27-signal Bangalore subset with {len(seeds)} random seeds &mdash; useful for detecting whether
the fine-tuning is directionally working and roughly how large the gap to CoSiCoSt's reported benefit is,
not a claim of equivalent real-world validation.</i></p>
"""

    report_path = out_dir / "scaleup_report.html"
    with open(report_path, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"\nReport written to: {report_path}")


if __name__ == "__main__":
    main()
