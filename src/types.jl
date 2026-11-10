# types.jl
#
# Plain data types shared by the whole package:
#   - per-swimmer state: position, heading, pace, mode (free / following + which leader)
#   - full state: agent swimmer, water current, field swimmers, elapsed time
#   - action: sight or not, turn (-Δ, 0, +Δ), follow or solo
#   - observation: buoy bearing (only when sighting), noisy field positions, draft status
#
# Should be cheap to copy (immutable, ideally fixed-size) because planners create
# millions of states. Also defines the discrete action set and which actions are
# legal in a given state (follow only when a leader is in the draft zone).
