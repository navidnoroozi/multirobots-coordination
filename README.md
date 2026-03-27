# Multi-Agent Consensus & Formation Control (Python Implementations)

This repository contains Python implementations of distributed control algorithms for multi-agent systems, aligned with the theoretical developments documented in the corresponding Overleaf project:

👉 https://www.overleaf.com/project/696105b7d2062b6cca97fb55

The focus is on **behavior-based multi-robots coordination (consensus/formation) control**, including extensions with **collision avoidance** and **obstacle avoidance**.

---

## 📌 Overview

This project bridges **theoretical results** and **practical implementations** for multi-robot coordination problems.

Implemented scenarios include:

* ✅ Consensus
* ✅ Consensus with inter-agent collision avoidance
* ✅ Formation control
* ✅ Formation control with:
  * Inter-agent collision avoidance
  * Object/obstacle avoidance

All algorithms are designed to closely follow the mathematical formulations presented in the accompanying LaTeX sources.

---

## 🧠 Theoretical Background

The implementations are derived from the theoretical framework documented in the Overleaf project. Key topics include:

* Graph-based consensus / formation protocols
* Communication-aware distributed control laws
* Layered control architecture
    * High-level hybrid / supervisory control
    * Low-level consensus / formation / safety controllers 
* Potential-field-based collision avoidance
* Formation stabilization
* Obstacle avoidance strategies

Refer to the Overleaf document for detailed derivations, proofs, and assumptions.

---

## 🗂️ Repository Structure

```
.
├── 0_consensus_no_lex/
│   ├── .../
│   ├── .../
│   ├── .../
│   ├── .../
│   └── .../
│
├── 1_consensus_with_lex_with_state_as_opt_variable/
│   ├── .../
│   ├── .../
│   ├── .../
│   ├── .../
│   └── .../
│
.
.
.
├── 7_formation_obj_col_avoid_bcf/
│   └── .../
│
├── requirements.txt
└── README.md
```

> Note: Adjust structure as needed depending on your actual implementation.

---

## ⚙️ Installation

Clone the repository:

```bash
git clone https://github.com/navidnoroozi/multirobots-coordination
```

Create a virtual environment (recommended):

```bash
python -m venv venv
source venv/bin/activate   # Linux / Mac
venv\Scripts\activate      # Windows
```

Install dependencies:

```bash
pip install -r requirements.txt
```

---

## 🚀 Usage

Run example simulations:

### Consensus

```bash
python simulations/consensus_demo.py
```

### Formation Control

```bash
python simulations/formation_demo.py
```

### Combined Scenario (Formation + Avoidance)

```bash
python simulations/combined_scenarios.py
```

---

## 🧩 Features

### 1. Consensus

* Distributed MPC
* Graph Laplacian-based updates
* Supports arbitrary communication topologies

### 2. Collision Avoidance

* Inter-agent repulsive potentials
* Safe distance enforcement
* Compatible with consensus & formation

### 3. Formation Control

* Shape stabilization via relative positions
* Leader-follower / distributed variants
* Scalable to multiple agents

### 4. Obstacle Avoidance

* Static obstacle modeling
* Repulsive field methods
* Seamless integration with formation control

---

## 📊 Visualization

Simulations can include:

* Agent trajectories
* Formation convergence
* Collision avoidance behavior
* Obstacle interaction

Outputs may be:

* Matplotlib plots
* Animations
* Saved simulation logs

---

## 🔧 Configuration

Most simulations can be configured via:

* Number of agents
* Graph topology
* Control gains
* Safety distances
* Obstacle parameters

---

## 📚 Dependencies

Typical dependencies include:

```
numpy
cvxpy
scipy
matplotlib
zmq
```

(Adjust based on your actual implementation)

---

## 🧪 Reproducibility

To reproduce results:

1. Use parameters specified in the Overleaf document
2. Run corresponding simulation scripts
3. Compare trajectories and convergence behavior

---

## 🤝 Contributing

Contributions are welcome, especially for:

* New control strategies
* Improved numerical stability
* Better visualization tools
* Benchmark scenarios

Please open an issue or submit a pull request.

---

## 📄 License

No licenses at the moment.

---

## 👤 Author

* Navid Noroozi

---

## 🔗 Related Work

* Overleaf Project (theoretical reference)
* Relevant papers / publications (optional)

---

## ⚠️ Disclaimer

This repository is intended for **research and educational purposes**.
Performance and safety in real-world systems are not guaranteed without further validation.

---
