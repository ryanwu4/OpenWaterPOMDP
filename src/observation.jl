# observation.jl
#
# observation(m, s, a, sp): the distribution over what the agent sees after a cycle.
#   - sighting: buoy bearing atan2(-y, L-x) - ψ + η, a wrapped angle
#   - field swimmer positions with noise
#   - drafting status, always available
# Must support both sampling and density evaluation: particle filters and POMCPOW
# weight particles by pdf. The density must never be exactly zero for plausible
# particles, e.g. a deterministic draft-status component needs softening.
#
# For POMCPOW: identical observations (e.g. no-sight steps with the same draft status)
# must compare equal so the tree reuses branches. Keep the observation low-dimensional,
# e.g. only nearby field swimmers, so likelihoods don't become so peaked that most
# particles get near-zero weight.
