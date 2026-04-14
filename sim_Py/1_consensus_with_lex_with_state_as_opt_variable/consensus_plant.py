# consensus_plant.py
import csv
from pathlib import Path

import numpy as np
import zmq

from consensus_config import NetConfig
from consensus_comm import make_envelope, dumps, loads

def clip(x, lo, hi):
    return np.minimum(np.maximum(x, lo), hi)

def main():
    cfg = NetConfig()
    ctx = zmq.Context.instance()

    req = ctx.socket(zmq.REQ)
    req.connect(cfg.plant_to_coord_rep)
    print(f"[Plant] Connected REQ -> Coordinator at {cfg.plant_to_coord_rep}")

    # initial conditions (spread out)
    z = np.array([
        [-4.0,  2.0],
        [ 3.5,  4.0],
        [ 4.5, -3.5],
        [-2.5, -4.0],
    ], dtype=float)

    log_path = Path("consensus_log.csv")
    with log_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "outer_j", "inner_k",
            "agent", "x0", "x1",
            "u0", "u1",
            "cost",
            "res0", "res1",
            "t_min_weight",
            "lex_used",
            "alpha_gamma",
        ])

        inner_k = 0
        for j in range(cfg.outer_steps):
            # Ask coordinator for M-step open-loop sequence
            req.send(dumps(make_envelope("plant_step", {
                "outer_index": j,
                "z_all": z.tolist()
            }, src="plant", dst="coord")))
            reply = loads(req.recv())["payload"]

            if not reply.get("ok", False):
                print(f"[Plant] Warning: infeasible at outer step {j}. Using returned fallback U.")
            U = np.array(reply["U_seq"], dtype=float)  # (M,n,dim)

            costs = reply.get("costs", {})
            residuals = reply.get("terminal_residuals", {})
            tmins = reply.get("t_min_weight", {})
            lex_used = reply.get("lex_used", {})
            alpha_gamma = float(reply.get("alpha_gamma", cfg.alpha_gamma))

            # simulate M inner steps open-loop
            for t in range(cfg.M):
                u = U[t, :, :]
                u = clip(u, cfg.u_min, cfg.u_max)

                # integrator dynamics
                z = z + u
                z = clip(z, cfg.x_min, cfg.x_max)

                for i in range(cfg.n_agents):
                    # dict keys can be int or str depending on JSON roundtrip
                    key_int = i + 1
                    key_str = str(i + 1)

                    cost_i = float(costs.get(key_str, costs.get(key_int, np.nan))) if isinstance(costs, dict) else np.nan
                    res_i = residuals.get(key_str, residuals.get(key_int, [np.nan, np.nan])) if isinstance(residuals, dict) else [np.nan, np.nan]
                    t_i = float(tmins.get(key_str, tmins.get(key_int, np.nan))) if isinstance(tmins, dict) else np.nan
                    lex_i = bool(lex_used.get(key_str, lex_used.get(key_int, False))) if isinstance(lex_used, dict) else False

                    w.writerow([
                        j, inner_k,
                        i + 1, z[i, 0], z[i, 1],
                        u[i, 0], u[i, 1],
                        cost_i,
                        res_i[0], res_i[1],
                        t_i,
                        int(lex_i),
                        alpha_gamma,
                    ])
                inner_k += 1

            if (j + 1) % 5 == 0:
                print(f"[Plant] Completed outer step {j+1}/{cfg.outer_steps}")

    print(f"[Plant] Done. Log written to: {log_path.resolve()}")

    # Shutdown coordinator and controllers cleanly
    req.send(dumps(make_envelope("shutdown", {}, src="plant", dst="coord")))
    _ = loads(req.recv())
    print("[Plant] Shutdown complete.")

if __name__ == "__main__":
    main()
