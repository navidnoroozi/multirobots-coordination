OPTION 1:
🤖 PLANNING (Python) <MATLAB API Engive for Python> MOTOIN-CONTROL (MATLAB/Simulink):
Plannaining Layer: 
  - Formation+Obstacle Avoidance+Collision Avoidance
Motion-Control:
  - Unicycle Model + PI Controller

HOW TO RUN:

  - Planning+Motion-Coontrol
python cosim_experiment_runner.py --n-agents 4 --model single_integrator --outer-steps 500 --cosim-mode matlab

  - Planning
python cosim_experiment_runner.py --n-agents 4 --model single_integrator --outer-steps 500 --cosim-mode dry-run



OPTION 2 - ONLY PYTHON:
HOW TO RUN THE PYTHON ENVIROMENT:

▶️single_integrator (SI) case:
python experiment_runner.py --n-agents 4 --model single_integrator --outer-steps 30 --use-lexicographic true --safety-enabled true --obstacles-enabled true

▶️double_integrator (DI) case:
python experiment_runner.py --n-agents 4 --model double_integrator --outer-steps 30 --use-lexicographic true --safety-enabled true --obstacles-enabled true

If you also want to regenerate the offline certificate first:
python experiment_runner.py --mode star --feasible --recompute_offline

A fuller example:
python experiment_runner.py ^
  --mode star ^
  --feasible ^
  --recompute_offline ^
  --offline_json offline_dcmg_fsclf.json ^
  --steps 2000 ^
  --log_out sim_log.jsonl ^
  --offline_ts 1e-5 ^
  --offline_mmax 2000 ^
  --offline_qV 1e-2 ^
  --offline_rU 1e-2 ^
  --offline_eps_gain 1.0