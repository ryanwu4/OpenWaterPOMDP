# planners.jl
#
# The online planner: POMCPOW on the continuous problem (the only planner).
# POMCPOW's defaults (random new actions, random rollouts) don't work for a
# ~460-step race with a hybrid action, so this file provides:
#   - action sampler (next_action): sight / follow / continuous turn δ, with δ centered
#     on a heuristic aim and "follow" offered only when a draft is possible
#   - leaf value (estimate_value): rollout with a tuned baseline, or a closed-form
#     estimate of remaining race time
#   - solver settings: query budget / time, depth, exploration constant, action and
#     observation widening
# The action sampler and leaf value are shared with the oracle (oracle.jl), so the two
# planners differ only in what they know.
