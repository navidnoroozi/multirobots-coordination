from __future__ import annotations

import argparse
import csv
from pathlib import Path

import zmq

from consensus_comm import loads
from consensus_config import add_common_args, config_from_namespace


def main() -> None:
    parser = add_common_args(argparse.ArgumentParser())
    ns = parser.parse_args()
    cfg = config_from_namespace(ns)

    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.PULL)
    sock.setsockopt(zmq.LINGER, cfg.req_linger_ms)
    sock.bind(cfg.logger_endpoint)
    print(f"[Logger] PULL bound at {cfg.logger_endpoint}")

    traj_path = Path("consensus_traj.csv")
    metrics_path = Path("consensus_metrics.csv")
    outer_path = Path("consensus_outer.csv")
    modes_path = Path("hybrid_modes.csv")

    with traj_path.open("w", newline="") as f_traj, \
         metrics_path.open("w", newline="") as f_met, \
         outer_path.open("w", newline="") as f_out, \
         modes_path.open("w", newline="") as f_mode:
        w_traj = csv.writer(f_traj)
        w_met = csv.writer(f_met)
        w_out = csv.writer(f_out)
        w_mode = csv.writer(f_mode)

        w_traj.writerow(["k_global", "outer_j", "inner_t", "agent", "r0", "r1", "v0", "v1", "u0", "u1"])
        w_met.writerow(["k_global", "outer_j", "inner_t", "Vr", "Vv", "max_speed", "mean_speed", "min_pairwise_distance", "n_mode1", "n_mode2", "n_mode3", "n_switched"])
        w_out.writerow(["outer_j", "agent", "model", "M", "objective_primary", "lex_used", "phi_terminal", "ri_terminal", "r_term0", "r_term1", "diam_C", "t_primary_ms", "t_lex_ms", "t_total_ms", "n_var_primary", "n_eq_primary", "n_ineq_primary", "n_var_lex", "n_eq_lex", "n_ineq_lex"])
        w_mode.writerow(["k_global", "outer_j", "inner_t", "agent", "mode", "mode_name", "switched", "mode_steps", "dmin_local", "local_diameter", "n_warn_neighbors", "anchor_norm"])

        while True:
            msg = loads(sock.recv())
            mtype = msg.get("type")
            p = msg.get("payload", {})
            if mtype == "shutdown":
                print("[Logger] Shutdown requested.")
                break
            if mtype == "log_traj":
                w_traj.writerow(p["row"])
                continue
            if mtype == "log_metrics":
                w_met.writerow(p["row"])
                continue
            if mtype == "log_outer":
                w_out.writerow(p["row"])
                continue
            if mtype == "log_mode":
                w_mode.writerow(p["row"])
                continue


if __name__ == "__main__":
    main()