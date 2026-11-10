# baselines.jl
#
# Rule-based policies for the agent, as POMDPs.jl Policy types:
#   - sighting: sight every N strokes and re-aim at the buoy
#   - drafting: never, always, or for a fixed duration
# Parameterized (N, duration) so they can be tuned per condition. Also used as
# rollout policies by the planners.
