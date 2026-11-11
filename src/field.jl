# field.jl
#
# Sight events and the rule-based field behavior that runs inside every transition:
#   - field swimmers sight on their own schedule (every N_k strokes, as a timer in seconds),
#     pause for τκ, then re-aim at the buoy with noise σ_aim (no current compensation)
#   - field swimmers don't sight while following; on release a sight is due at once
#   - the agent's sight is scheduled by its action (after τ0 of swimming) and doesn't re-aim
# Drafting decisions for the field (draft faster swimmers, fixed duration, cooldown) are
# in drafting.jl.

# Whether swimmer i's sight timer is running.
sight_counting(sw::Swimmer, i::Int) =
    !sw.finished && !pausing(sw) && (i == AGENT || !following(sw)) && isfinite(sw.sight_timer)

"""
    end_pause(p, s, i, rng)

Swimmer `s` (index i) has just finished its sight pause. Field swimmers re-aim at the
buoy and restart their sight schedule; the agent just resumes swimming.
"""
function end_pause(p::SwimParams, s::Swimmer, i::Int, rng::AbstractRNG)
    i == AGENT && return setproperties(s, (pause = 0.0, sight_timer = Inf))
    ψ = wrap(bearing_to_buoy(p, s.x, s.y) + p.σ_aim * randn(rng))
    return setproperties(s, (pause = 0.0, ψ = ψ, sight_timer = s.sight_period))
end

"""Start sight pauses whose timers have run out."""
function start_sights!(p::SwimParams, sw::AbstractVector{Swimmer})
    for i in eachindex(sw)
        s = sw[i]
        (sight_counting(s, i) && s.sight_timer <= EPS) || continue
        sw[i] = setproperties(s, (pause = p.τκ, sight_timer = Inf))
    end
    return
end
