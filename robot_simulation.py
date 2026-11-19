"""Command-line entry point for the CyberKnife tracking project."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from src.live_demo import launch_live_demo
from src.simulation import (
    EXPERIMENTS,
    _write_plot,
    _write_summary,
    run_experiment,
    run_experiments,
)


PROJECT_ROOT = Path(__file__).resolve().parent


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Simulate a six-DoF arm tracking a breathing-driven target."
    )
    parser.add_argument(
        "--experiment",
        choices=("all", *EXPERIMENTS.keys()),
        default="all",
        help="Experiment to run (default: all five headless experiments).",
    )
    parser.add_argument(
        "--view",
        action="store_true",
        help="Open the continuous viewer and interactive joint/trajectory controls.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=PROJECT_ROOT / "results",
        help="Directory for CSV metrics and plots (default: ./results).",
    )
    args = parser.parse_args()
    output_dir = args.output_dir.resolve()

    if args.view:
        if args.experiment != "all":
            parser.error("--experiment applies to headless runs; use the live buttons instead.")
        launch_live_demo(PROJECT_ROOT / "cyberknife_6dof.xml")
        print("Live simulation closed.")
        return
    elif args.experiment == "all":
        summaries = run_experiments(PROJECT_ROOT, output_dir)
    else:
        summary = run_experiment(
            PROJECT_ROOT / "cyberknife_6dof.xml",
            output_dir,
            EXPERIMENTS[args.experiment],
        )
        _write_summary(output_dir, [summary])
        _write_plot(output_dir, [summary])
        summaries = [summary]

    print(json.dumps(summaries, indent=2))
    print(f"\nResults saved to: {output_dir}")


if __name__ == "__main__":
    main()
