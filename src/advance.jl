# advance.jl
#
# advance(world, T, rng): move the whole race forward by T seconds.
# Splits the interval at every swimmer's scheduled events (sight pauses, re-aims,
# draft timers, cooldowns) and at regular contact-check ticks. In each segment:
# move solo swimmers, then followers in dependency order, then check finishes.
# At each boundary: process events and draft bookkeeping (field.jl, drafting.jl).
# Stops early and reports the finish time if the agent finishes.
#
# gen is advance by the agent's Δt. A multi-agent scheduler would call it
# between swimmers' decision times.
