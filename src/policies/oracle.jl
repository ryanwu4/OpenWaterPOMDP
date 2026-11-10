# oracle.jl
#
# Full-information policy: MCTS with double progressive widening (MCTS.jl) on the fully
# observed MDP view of the problem, so it plans from the true state (including the
# current and every swimmer's per-race constants) instead of a belief.
# Uses the same action sampler, leaf value and search budget as POMCPOW (planners.jl),
# so the gap between the two measures the value of information, not planner differences.
