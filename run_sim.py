#!/usr/bin/env python
"""MICS scenario runner CLI.

Usage:
    py run_sim.py scenarios/happy_path.yaml
    py run_sim.py scenarios/evasive_reassign.yaml --seed 7
    py run_sim.py scenarios/allocation_stress.yaml --json
"""

from __future__ import annotations

import argparse
import json
import sys

from mics.config import load_scenario
from mics.sim import Simulation


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Run a MICS scenario (pure-Python core).")
    p.add_argument("scenario", help="path to a scenario YAML")
    p.add_argument("--seed", type=int, default=None, help="override scenario seed")
    p.add_argument("--json", action="store_true", help="print metrics as JSON")
    args = p.parse_args(argv)

    cfg = load_scenario(args.scenario)
    if args.seed is not None:
        cfg.seed = args.seed

    sim = Simulation(cfg)
    metrics = sim.run()
    summary = metrics.summary()

    if args.json:
        print(json.dumps(summary, indent=2))
    else:
        print(f"Scenario: {args.scenario}  (seed={cfg.seed}, mode={cfg.sensor_mode}, "
              f"alloc={cfg.allocation.algorithm}, defenders={cfg.n_defenders})")
        print("-" * 60)
        for k, v in summary.items():
            print(f"  {k:32s}: {v}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
