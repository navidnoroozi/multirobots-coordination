# plot_consensus.py
import pandas as pd
import matplotlib.pyplot as plt

def main():
    df = pd.read_csv("consensus_log.csv")

    # --- Trajectories (x0 vs x1) ---
    plt.figure()
    for agent in sorted(df["agent"].unique()):
        dfa = df[df["agent"] == agent].sort_values(["inner_k"])
        plt.plot(dfa["x0"].values, dfa["x1"].values, label=f"agent {agent}")
    plt.xlabel("x0")
    plt.ylabel("x1")
    plt.title("Agent trajectories in state space")
    plt.legend()
    plt.grid(True)

    # --- Convex-hull terminal residual norm over time (per agent) ---
    plt.figure()
    for agent in sorted(df["agent"].unique()):
        dfa = df[df["agent"] == agent].sort_values(["inner_k"])
        res_norm = (dfa["res0"]**2 + dfa["res1"]**2) ** 0.5
        plt.plot(dfa["inner_k"].values, res_norm.values, label=f"agent {agent}")
    plt.xlabel("inner step k")
    plt.ylabel("||terminal residual||")
    plt.title("Convex-hull terminal residual norm (should be ~0)")
    plt.yscale("log")
    plt.legend()
    plt.grid(True)

    # --- Cost per outer step (per agent) ---
    plt.figure()
    for agent in sorted(df["agent"].unique()):
        dfa = df[df["agent"] == agent].sort_values(["outer_j", "inner_k"])
        dfa_outer = dfa.groupby("outer_j", as_index=False).head(1)
        plt.plot(dfa_outer["outer_j"].values, dfa_outer["cost"].values, marker="o", label=f"agent {agent}")
    plt.xlabel("outer step j")
    plt.ylabel("Primary MPC objective value J*")
    plt.title("Per-agent MPC objective over outer steps")
    plt.legend()
    plt.grid(True)

    # --- Lexicographic min-weight (per outer step) ---
    plt.figure()
    for agent in sorted(df["agent"].unique()):
        dfa = df[df["agent"] == agent].sort_values(["outer_j", "inner_k"])
        dfa_outer = dfa.groupby("outer_j", as_index=False).head(1)
        plt.plot(dfa_outer["outer_j"].values, dfa_outer["t_min_weight"].values, marker="o", label=f"agent {agent}")
    plt.xlabel("outer step j")
    plt.ylabel("t = min_k a_k (selected)")
    plt.title("Lexicographic interiority proxy (higher -> more interior in barycentric coords)")
    plt.legend()
    plt.grid(True)

    plt.show()

if __name__ == "__main__":
    main()
