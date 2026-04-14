from __future__ import annotations

import argparse
import zmq
import numpy as np

from consensus_comm import dumps, loads, make_envelope
from consensus_config import add_common_args, config_from_namespace
from consensus_controller import solve_mpc_request
from hybrid_controller import HybridController


def main() -> None:
    parser = add_common_args(argparse.ArgumentParser())
    ns = parser.parse_args()

    if ns.agent_id is None:
        raise SystemExit("controller_node.py requires --agent-id")

    cfg = config_from_namespace(ns)
    agent_id = int(ns.agent_id)
    endpoint = cfg.controller_endpoint(agent_id)
    hybrid = HybridController(cfg, agent_id - 1)

    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.REP)
    sock.setsockopt(zmq.LINGER, cfg.req_linger_ms)
    sock.bind(endpoint)
    print(
        f"[ControllerNode {agent_id}] REP bound at {endpoint} | "
        f"model={cfg.model} | hybrid enabled={cfg.hybrid_enabled}"
    )

    while True:
        msg = loads(sock.recv())
        mtype = msg.get("type")

        if mtype == "shutdown":
            sock.send(dumps(make_envelope("shutdown_ack", {"ok": True}, src=f"controller_node{agent_id}")))
            break

        if mtype == "mpc_request":
            out, ok = solve_mpc_request(cfg, msg["payload"])
            if ok:
                sock.send(dumps(make_envelope("mpc_reply", out, src=f"controller_node{agent_id}")))
            else:
                sock.send(
                    dumps(
                        make_envelope(
                            "mpc_reply",
                            {"ok": False, "agent_id": agent_id, "reason": "infeasible_or_solver_failed"},
                            src=f"controller_node{agent_id}",
                        )
                    )
                )
            continue

        if mtype == "hybrid_request":
            p = msg["payload"]
            r = np.array(p["r_all"], dtype=float)
            v = np.array(p["v_all"], dtype=float)
            u_nom = np.array(p["u_nom_all"], dtype=float)
            result = hybrid.step(r, v, u_nom)
            sock.send(
                dumps(
                    make_envelope(
                        "hybrid_reply",
                        {
                            "ok": True,
                            "agent_id": agent_id,
                            "u_cmd": result.u_cmd.tolist(),
                            "diag": result.diag,
                        },
                        src=f"controller_node{agent_id}",
                    )
                )
            )
            continue

        sock.send(dumps(make_envelope("error", {"error": f"unknown msg type: {mtype}"}, src=f"controller_node{agent_id}")))


if __name__ == "__main__":
    main()