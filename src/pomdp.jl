# pomdp.jl
#
# The POMDPs.jl problem definition. A POMDP type holding a params instance, implementing:
#   - gen(m, s, a, rng): apply the agent's action, call advance by its Δt, return next state and
#     reward -τ_m (observation comes from observation.jl)
#   - actions, initialstate (samples current, paces, start positions from the params'
#     distributions), isterminal, discount
#
# Also a fully observed MDP view of the same dynamics, used by the full-information
# (oracle) policy.
