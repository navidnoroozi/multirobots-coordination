"""cosim_experiment_runner.py
Drop-in replacement for ``experiment_runner.py`` that launches the co-simulation
pipeline.

What changes vs. the original runner
-------------------------------------
* Launches ``cosim_manager.py`` instead of ``plant_node.py``.
* Passes ``--cosim-mode`` and Simulink-specific flags to the manager.
* Everything else (coordinator, per-agent controller nodes) is unchanged.

Usage
-----
Run with MATLAB/Simulink co-simulation (default):

    python cosim_experiment_runner.py \
        --n-agents 4 --model single_integrator --outer-steps 18 \
        --cosim-mode matlab --simulink-dt 0.01 --k-heading 4.0

Dry-run (planning layer only, MATLAB not started):

    python cosim_experiment_runner.py \
        --n-agents 4 --model single_integrator --outer-steps 18 \
        --cosim-mode dry-run
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

from consensus_config import add_common_args, config_from_namespace


def _bool_str(v: bool) -> str:
    return "true" if v else "false"


def _common_cli(ns: argparse.Namespace) -> list[str]:
    from consensus_config import config_from_namespace
    cfg = config_from_namespace(ns)
    return [
        "--n-agents",       str(cfg.n_agents),
        "--model",          cfg.model,
        "--outer-steps",    str(cfg.outer_steps),
        "--use-lexicographic", _bool_str(cfg.use_lexicographic),
        "--safety-enabled",    _bool_str(cfg.safety_enabled),
        "--safety-method",     cfg.safety_method,
        "--obstacles-enabled", _bool_str(cfg.obstacles_enabled),
        "--startup-delay-s",   str(cfg.startup_delay_s),
    ]


def _add_cosim_runner_args(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    parser.add_argument(
        "--cosim-mode",
        choices=["matlab", "dry-run"],
        default="matlab",
    )
    parser.add_argument("--simulink-dt", type=float, default=0.01)
    parser.add_argument("--k-heading", type=float, default=4.0)
    return parser


def main() -> None:
    parser = _add_cosim_runner_args(add_common_args(argparse.ArgumentParser()))
    ns = parser.parse_args()
    cfg = config_from_namespace(ns)

    common = _common_cli(ns)
    py = sys.executable
    here = Path(__file__).resolve().parent

    cosim_extra = [
        "--cosim-mode", ns.cosim_mode,
        "--simulink-dt", str(ns.simulink_dt),
        "--k-heading", str(ns.k_heading),
    ]

    procs: list[subprocess.Popen] = []
    try:
        print(
            f"[CosimRunner] n_agents={cfg.n_agents} model={cfg.model} "
            f"outer_steps={cfg.outer_steps} cosim_mode={ns.cosim_mode}"
        )

        # Coordinator
        procs.append(
            subprocess.Popen(
                [py, str(here / "coordinator_node.py"), *common],
                cwd=here,
            )
        )

        # Per-agent controller nodes (unchanged)
        for i in range(1, cfg.n_agents + 1):
            procs.append(
                subprocess.Popen(
                    [py, str(here / "controller_node.py"), *common, "--agent-id", str(i)],
                    cwd=here,
                )
            )

        time.sleep(cfg.startup_delay_s)

        # Co-sim manager (replaces plant_node)
        plant = subprocess.Popen(
            [py, str(here / "cosim_manager.py"), *common, *cosim_extra],
            cwd=here,
        )
        rc = plant.wait()
        print(f"[CosimRunner] Co-sim manager finished with return code {rc}.")

    finally:
        for p in procs:
            if p.poll() is None:
                p.terminate()
        for p in procs:
            try:
                p.wait(timeout=3.0)
            except Exception:
                if p.poll() is None:
                    p.kill()


if __name__ == "__main__":
    main()
