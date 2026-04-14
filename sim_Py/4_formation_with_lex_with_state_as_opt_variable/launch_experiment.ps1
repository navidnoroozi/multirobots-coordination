param(
    [int]$n_agents = 4,
    [ValidateSet('single_integrator','double_integrator')]
    [string]$model = 'single_integrator',
    [int]$outer_steps = 10,
    [string]$use_lexicographic = 'true'
)

python .\experiment_runner.py `
    --n-agents $n_agents `
    --model $model `
    --outer-steps $outer_steps `
    --use-lexicographic $use_lexicographic `
    --safety-enabled true `
    --safety-method safe_formation