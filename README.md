# OpenWaterPOMDP

Open-water swim race strategy (sight, draft, or swim blind?) as a POMDP. AA228/CS238 final project — Erika Hunting, Renee Nguyen, Ryan Wu.

## Setup

```bash
julia --project=. -e 'using Pkg; Pkg.instantiate()'
julia --project=. -e 'using Pkg; Pkg.test()'
```

For development, use a REPL with Revise: `julia --project=.`, then `using Revise, OpenWaterPOMDP`.

## Layout

```
src/                      the package (`using OpenWaterPOMDP`)
  OpenWaterPOMDP.jl       module entry point
  params.jl               model spec: all constants and priors
  types.jl                state / action / observation types
  dynamics.jl             single-swimmer motion, current, geometry
  drafting.jl             draft zone, leader following
  field.jl                rule-based field swimmers
  advance.jl              moves the whole race forward in time
  pomdp.jl                POMDPs.jl problem definition
  observation.jl          observation distribution
  belief.jl               particle filter
  policies/               baselines, oracle, planners
  metrics.jl              per-race statistics
test/
experiments/
analysis/
sandbox/                  scratch work
results/                  outputs (gitignored)
```

Each `src` file's header describes what it is responsible for.
