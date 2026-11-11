# advance.jl
#
# advance!(p, sw, c, T, rng): move the whole race forward by T seconds, in place.
# The interval is split at every swimmer's scheduled events (sight pauses, draft timers,
# cooldowns) and at contact-check ticks at most h_max apart. In each segment, solo
# swimmers move first, then followers in leader order, then finishes are checked. At
# each boundary, events and draft bookkeeping are processed.
# Stops early, at the exact time, if the agent finishes.
#
# step_race applies the agent's action and advances by its Δt; gen wraps it.
# A multi-agent scheduler would call advance! between swimmers' decision times.

"""Process everything that happens at a segment boundary, in a fixed order."""
function boundary!(p::SwimParams, sw::AbstractVector{Swimmer}, agent_wants::Bool)
    release_invalid!(p, sw)
    start_drafts!(p, sw, agent_wants)
    release_invalid!(p, sw)
    start_sights!(p, sw)
    return
end

"""Length of the next segment: up to the next scheduled event, at most h_max."""
function next_segment(p::SwimParams, sw::AbstractVector{Swimmer}, remaining::Float64)
    h = min(remaining, p.h_max)
    for i in eachindex(sw)
        s = sw[i]
        s.finished && continue
        pausing(s) && (h = min(h, s.pause))
        sight_counting(s, i) && s.sight_timer > EPS && (h = min(h, s.sight_timer))
        i != AGENT && following(s) && s.draft_timer > EPS && (h = min(h, s.draft_timer))
        s.cooldown > EPS && (h = min(h, s.cooldown))
    end
    return max(h, EPS)
end

"""Count every running timer down by h, ending sight pauses that run out."""
function tick!(p::SwimParams, sw::AbstractVector{Swimmer}, h::Float64, rng::AbstractRNG)
    for i in eachindex(sw)
        s = sw[i]
        s.finished && continue
        counting = sight_counting(s, i)
        was_pausing = pausing(s)
        s = setproperties(s, (
            pause = was_pausing ? s.pause - h : 0.0,
            sight_timer = counting ? s.sight_timer - h : s.sight_timer,
            draft_timer = (i != AGENT && following(s)) ? s.draft_timer - h : s.draft_timer,
            cooldown = max(0.0, s.cooldown - h),
        ))
        was_pausing && !pausing(s) && (s = end_pause(p, s, i, rng))
        sw[i] = s
    end
    return
end

"""
Move every unfinished swimmer through one segment of length h. Returns the fraction of
the segment at which the agent finished, or -1.0.
"""
function move_segment!(p::SwimParams, sw::MVector{N,Swimmer}, c::SVector{2,Float64},
                       h::Float64, rng::AbstractRNG) where {N}
    moved = MVector{N,Bool}(ntuple(i -> sw[i].finished, Val(N)))
    adv = MVector{N,Float64}(ntuple(_ -> 0.0, Val(N)))
    x0 = MVector{N,Float64}(ntuple(i -> sw[i].x, Val(N)))
    y0 = MVector{N,Float64}(ntuple(i -> sw[i].y, Val(N)))

    for i in 1:N                                  # solo swimmers
        (moved[i] || following(sw[i])) && continue
        sw[i], adv[i] = move_solo(p, sw[i], c, h, rng)
        moved[i] = true
    end
    for _ in 1:N                                  # followers, once their leader has moved
        progress = false
        for i in 1:N
            moved[i] && continue
            j = sw[i].leader
            moved[j] || continue
            sw[i], adv[i] = move_follower(p, sw[i], sw[j], adv[j], h)
            moved[i] = progress = true
        end
        progress || break
    end

    agent_frac = -1.0
    for i in 1:N                                  # finishes along each path
        s = sw[i]
        s.finished && continue
        f = finish_fraction(p, x0[i], y0[i], s.x, s.y)
        f < 0 && continue
        x = x0[i] + f * (s.x - x0[i])
        y = y0[i] + f * (s.y - y0[i])
        sw[i] = setproperties(s, (x = x, y = y, finished = true, pause = 0.0))
        i == AGENT && (agent_frac = f)
    end
    return agent_frac
end

"""
    advance!(p, sw, c, T, rng; agent_wants = false) -> (elapsed, agent_finished)

Advance the race by T seconds in place. `agent_wants` lets the agent start a draft at
the first boundary (the start of its step).
"""
function advance!(p::SwimParams, sw::MVector{N,Swimmer}, c::SVector{2,Float64}, T::Float64,
                  rng::AbstractRNG; agent_wants::Bool = false) where {N}
    boundary!(p, sw, agent_wants)
    elapsed = 0.0
    while elapsed < T - EPS
        h = next_segment(p, sw, T - elapsed)
        f = move_segment!(p, sw, c, h, rng)
        if f >= 0
            release_invalid!(p, sw)
            return elapsed + f * h, true
        end
        tick!(p, sw, h, rng)
        elapsed += h
        boundary!(p, sw, false)
    end
    return elapsed, false
end

"""
    step_race(p, s, a, rng) -> (sp, r)

One agent decision: apply the action (solo/follow choice, turn, sight schedule), advance
by Δt = τ0 + τκ·sight, and return the next state and reward −(elapsed time), with the
time-cap penalty on timeout.
"""
function step_race(p::SwimParams, s::SwimState{N}, a::SwimAction, rng::AbstractRNG) where {N}
    isterminal_state(p, s) && return s, 0.0
    sw = MVector(s.swimmers)
    a.follow || release!(p, sw, AGENT)
    ag = sw[AGENT]
    ψ = following(ag) ? ag.ψ : wrap(ag.ψ + clamp_turn(p, a.turn))
    sw[AGENT] = setproperties(ag, (ψ = ψ, sight_timer = a.sight ? p.τ0 : Inf))

    T = p.τ0 + (a.sight ? p.τκ : 0.0)
    elapsed, _ = advance!(p, sw, s.c, T, rng; agent_wants = a.follow)
    sp = SwimState(s.t + elapsed, s.c, SVector(sw))

    r = -elapsed
    ag = sw[AGENT]
    if !ag.finished && sp.t >= p.t_cap - EPS
        r -= dist_to_buoy(p, ag.x, ag.y) / ag.v
    end
    return sp, r
end

isterminal_state(p::SwimParams, s::SwimState) = agent(s).finished || s.t >= p.t_cap - EPS
