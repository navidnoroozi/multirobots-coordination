# summarize_perf.py
from __future__ import annotations

import csv
from collections import defaultdict
from pathlib import Path
from statistics import mean, median


def _to_float(x: str):
    try:
        v = float(x)
        if v != v:  # NaN
            return None
        return v
    except Exception:
        return None


def _to_int(x: str, default: int = 0) -> int:
    try:
        return int(float(x))
    except Exception:
        return default


def main() -> None:
    path = Path("consensus_outer.csv")
    if not path.exists():
        raise FileNotFoundError("consensus_outer.csv not found. Run the simulation first.")

    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    # Group by agent
    by_agent = defaultdict(list)
    for r in rows:
        a = _to_int(r.get("agent", "0"))
        if a > 0:
            by_agent[a].append(r)

    agents = sorted(by_agent.keys())
    if not agents:
        print("No agent data found.")
        return

    print("=== Performance Summary from consensus_outer.csv ===")

    # 1) mean/median/max solve times per agent
    print("\n-- Solve times per agent (ms): mean / median / max --")
    for a in agents:
        ttot = [_to_float(r.get("t_total_ms", "")) for r in by_agent[a]]
        ttot = [v for v in ttot if v is not None]
        tprim = [_to_float(r.get("t_primary_ms", "")) for r in by_agent[a]]
        tprim = [v for v in tprim if v is not None]
        tlex = [_to_float(r.get("t_lex_ms", "")) for r in by_agent[a]]
        tlex = [v for v in tlex if v is not None]

        def stats(vals):
            if not vals:
                return None
            return mean(vals), median(vals), max(vals)

        s_tot = stats(ttot)
        s_prim = stats(tprim)
        s_lex = stats(tlex)

        if s_tot is None:
            print(f"agent {a}: no timing data (NaNs)")
            continue

        mt, medt, maxt = s_tot
        mp, medp, maxp = s_prim if s_prim is not None else (float("nan"), float("nan"), float("nan"))
        ml, medl, maxl = s_lex if s_lex is not None else (float("nan"), float("nan"), float("nan"))

        print(
            f"agent {a}: "
            f"T_total={mt:.3f}/{medt:.3f}/{maxt:.3f} | "
            f"T_primary={mp:.3f}/{medp:.3f}/{maxp:.3f} | "
            f"T_lex={ml:.3f}/{medl:.3f}/{maxl:.3f}"
        )

    # 2) percentage of outer steps where lex_used=1
    print("\n-- Lex activation (percentage of outer steps with lex_used=1) --")
    for a in agents:
        lex = [_to_int(r.get("lex_used", "0"), 0) for r in by_agent[a]]
        total = len(lex)
        if total == 0:
            print(f"agent {a}: no data")
            continue
        pct = 100.0 * (sum(1 for v in lex if v == 1) / total)
        print(f"agent {a}: {pct:.2f}% ({sum(1 for v in lex if v == 1)}/{total})")

    # 3) average size metrics
    print("\n-- Average size metrics per agent (primary and lex) --")
    for a in agents:
        def avg_int_field(field: str):
            vals = [_to_int(r.get(field, "0"), 0) for r in by_agent[a]]
            return sum(vals) / len(vals) if vals else 0.0

        nvp = avg_int_field("n_var_primary")
        neqp = avg_int_field("n_eq_primary")
        ninp = avg_int_field("n_ineq_primary")

        nvl = avg_int_field("n_var_lex")
        neql = avg_int_field("n_eq_lex")
        ninl = avg_int_field("n_ineq_lex")

        print(
            f"agent {a}: "
            f"primary avg (n_var,n_eq,n_ineq)=({nvp:.1f},{neqp:.1f},{ninp:.1f}) | "
            f"lex avg (n_var,n_eq,n_ineq)=({nvl:.1f},{neql:.1f},{ninl:.1f})"
        )

    print("\nDone.")


if __name__ == "__main__":
    main()