# consensus_plant.py
import csv
from pathlib import Path
from typing import Dict, Any

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
            "eta_used",
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
            eta_used = reply.get("eta_used", {})

            # simulate M inner steps open-loop
            for t in range(cfg.M):
                u = U[t, :, :]
                # apply hard input bounds (just in case)
                u = clip(u, cfg.u_min, cfg.u_max)

                # integrator dynamics
                z = z + u
                # apply hard state bounds
                z = clip(z, cfg.x_min, cfg.x_max)

                # log per agent each inner step
                for i in range(cfg.n_agents):
                    cost_i = float(costs.get(str(i+1), costs.get(i+1, np.nan))) if isinstance(costs, dict) else np.nan
                    res_i = residuals.get(str(i+1), residuals.get(i+1, [np.nan, np.nan])) if isinstance(residuals, dict) else [np.nan, np.nan]
                    eta_i = float(eta_used.get(str(i+1), eta_used.get(i+1, np.nan))) if isinstance(eta_used, dict) else np.nan
                    w.writerow([
                        j, inner_k,
                        i + 1, z[i, 0], z[i, 1],
                        u[i, 0], u[i, 1],
                        cost_i,
                        res_i[0], res_i[1],
                        eta_i,
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
