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
    cfg = config_from_namespace(ns)
    return [
        "--n-agents", str(cfg.n_agents),
        "--model", cfg.model,
        "--outer-steps", str(cfg.outer_steps),
        "--use-lexicographic", _bool_str(cfg.use_lexicographic),
        "--safety-enabled", _bool_str(cfg.safety_enabled),
        "--safety-method", cfg.safety_method,
        "--startup-delay-s", str(cfg.startup_delay_s),
    ]


def main() -> None:
    parser = add_common_args(argparse.ArgumentParser())
    ns = parser.parse_args()
    cfg = config_from_namespace(ns)

    common = _common_cli(ns)
    py = sys.executable
    here = Path(__file__).resolve().parent

    procs: list[subprocess.Popen] = []
    try:
        print(
            f"[Runner] Starting experiment with n_agents={cfg.n_agents}, model={cfg.model}, "
            f"outer_steps={cfg.outer_steps}, use_lexicographic={cfg.use_lexicographic}, "
            f"objective_mode={cfg.objective_mode}."
        )

        procs.append(subprocess.Popen([py, str(here / "consensus_coordinator.py"), *common], cwd=here))
        for i in range(1, cfg.n_agents + 1):
            procs.append(subprocess.Popen([py, str(here / "controller_node.py"), *common, "--agent-id", str(i)], cwd=here))

        time.sleep(cfg.startup_delay_s)

        plant = subprocess.Popen([py, str(here / "consensus_plant.py"), *common], cwd=here)
        rc = plant.wait()
        print(f"[Runner] Plant finished with return code {rc}.")
    finally:
        for p in procs:
            if p.poll() is None:
                p.terminate()
        for p in procs:
            try:
                p.wait(timeout=2.0)
            except Exception:
                if p.poll() is None:
                    p.kill()


if __name__ == "__main__":
    main()