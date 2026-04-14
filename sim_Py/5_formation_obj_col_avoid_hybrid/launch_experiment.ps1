param(
    [int]$n_agents = 4,
    [ValidateSet('single_integrator','double_integrator')]
    [string]$model = 'single_integrator',
    [int]$outer_steps = 18,
    [string]$use_lexicographic = 'true',
    [string]$obstacles_enabled = 'true'
)

python .\experiment_runner.py `
    --n-agents $n_agents `
    --model $model `
    --outer-steps $outer_steps `
    --use-lexicographic $use_lexicographic `
    --safety-enabled true `
    --safety-method explicit_hybrid `
    --obstacles-enabled $obstacles_enabled