# drafting.jl
#
# Interactions between swimmers through drafting:
#   - draft-zone geometry (behind the leader only)
#   - who may follow whom: one follower per leader; leaders must be solo, except the
#     agent, which may be a middle link (field → agent → field)
#   - starting drafts, nearest-wins when several swimmers want the same leader
#   - releases (gap too large, leader finished or following, timer expired, by choice)
#   - locked-follower motion with the catch-up gap rule
#   - an invariant checker for all of the above

const EPS = 1e-9

"""
    zone_gap(p, f, l)

Along-track gap of follower `f` behind leader `l` if `f` is inside `l`'s draft zone,
otherwise `nothing`.
"""
function zone_gap(p::SwimParams, f::Swimmer, l::Swimmer)
    dx, dy = l.x - f.x, l.y - f.y
    ux, uy = cos(l.ψ), sin(l.ψ)
    along = dx * ux + dy * uy
    lat = dy * ux - dx * uy
    return (0.0 <= along <= p.g_max && abs(lat) <= p.w_lat) ? along : nothing
end

"""Index of each swimmer's follower (0 if none)."""
function followers(sw::StaticVector{N,Swimmer}) where {N}
    fol = MVector{N,Int}(ntuple(_ -> 0, Val(N)))
    for i in eachindex(sw)
        j = sw[i].leader
        j != 0 && (fol[j] = i)
    end
    return fol
end

"""Release swimmer `i` from its leader. Field swimmers enter cooldown and sight immediately."""
function release!(p::SwimParams, sw::AbstractVector{Swimmer}, i::Int)
    s = sw[i]
    following(s) || return
    if i == AGENT
        sw[i] = setproperties(s, (leader = 0, gap = 0.0))
    else
        sw[i] = setproperties(s, (leader = 0, gap = 0.0, draft_timer = Inf,
                                  cooldown = p.draft_cooldown, sight_timer = 0.0))
    end
    return
end

"""Release every follow relation that is no longer allowed."""
function release_invalid!(p::SwimParams, sw::AbstractVector{Swimmer})
    for i in eachindex(sw)
        s = sw[i]
        following(s) || continue
        l = sw[s.leader]
        if l.finished || s.finished || s.gap > p.g_max ||
           (s.leader != AGENT && following(l)) ||
           (i != AGENT && s.draft_timer <= EPS)
            release!(p, sw, i)
        end
    end
    return
end

can_lead(sw, j) = !sw[j].finished && (j == AGENT || !following(sw[j]))

# Whether swimmer i wants (and is allowed) to start following someone this boundary.
function wants_draft(sw, i, agent_wants)
    s = sw[i]
    (s.finished || following(s)) && return false
    i == AGENT && return agent_wants
    return !pausing(s) && s.cooldown <= EPS
end

# Leader j is acceptable to swimmer i under i's own rule.
function acceptable_leader(sw, i, j)
    i == j && return false
    i == AGENT && return j != AGENT                 # agent follows any field swimmer
    return sw[j].v > sw[i].v                        # field drafts faster swimmers only
end

"""
    start_drafts!(p, sw, agent_wants)

Start new drafts. Every swimmer that wants to draft proposes to each acceptable leader
whose zone it is in; proposals are accepted nearest-first, so a contested leader goes to
the closest swimmer. A field swimmer that starts following releases its own follower.
"""
function start_drafts!(p::SwimParams, sw::AbstractVector{Swimmer}, agent_wants::Bool)
    fol = followers(sw)
    N = length(sw)
    while true
        best_g, bi, bj = Inf, 0, 0
        for i in 1:N
            wants_draft(sw, i, agent_wants) || continue
            for j in 1:N
                (acceptable_leader(sw, i, j) && can_lead(sw, j) && fol[j] == 0) || continue
                g = zone_gap(p, sw[i], sw[j])
                g === nothing && continue
                g < best_g && ((best_g, bi, bj) = (g, i, j))
            end
        end
        bi == 0 && return
        if bi != AGENT && fol[bi] != 0              # field swimmer drops its follower
            release!(p, sw, fol[bi])
            fol[bi] = 0
        end
        s = sw[bi]
        timer = bi == AGENT ? Inf : s.draft_duration
        sw[bi] = setproperties(s, (leader = bj, gap = max(p.g_nom, best_g),
                                   draft_timer = timer,
                                   sight_timer = bi == AGENT ? s.sight_timer : Inf))
        fol[bj] = bi
    end
end

"""
    move_follower(p, f, l, a_lead, h) -> (swimmer, through_water_distance)

Locked follower motion over a segment of length h. `a_lead` is the leader's
through-water distance in the segment. The gap closes at (1+β)v and never below g_nom.
"""
function move_follower(p::SwimParams, f::Swimmer, l::Swimmer, a_lead::Float64, h::Float64)
    t_swim = pausing(f) ? 0.0 : h
    g = max(p.g_nom, f.gap + a_lead - (1 + p.β) * f.v * t_swim)
    x = l.x - g * cos(l.ψ)
    y = l.y - g * sin(l.ψ)
    a_self = a_lead - (g - f.gap)
    return setproperties(f, (x = x, y = y, ψ = l.ψ, gap = g)), a_self
end

"""
    check_invariants(p, s)

Return `nothing` if the drafting state is consistent, otherwise a description of the
first violation found.
"""
function check_invariants(p::SwimParams, s::SwimState)
    sw = s.swimmers
    fol = zeros(Int, length(sw))
    for i in eachindex(sw)
        f = sw[i]
        abs(f.ψ) <= π + EPS || return "swimmer $i heading $(f.ψ) not wrapped"
        f.finished && following(f) && return "finished swimmer $i still following"
        if i != AGENT && !f.finished && !following(f) && !pausing(f) && !isfinite(f.sight_timer)
            return "solo field swimmer $i has no sight scheduled"
        end
        abs(f.pause) <= p.τκ + EPS || return "swimmer $i pause $(f.pause) out of range"
        following(f) || continue
        j = f.leader
        j == i && return "swimmer $i follows itself"
        fol[j] != 0 && return "leader $j has followers $(fol[j]) and $i"
        fol[j] = i
        l = sw[j]
        l.finished && return "swimmer $i follows finished swimmer $j"
        j != AGENT && following(l) && return "swimmer $i follows $j, which is following"
        p.g_nom - EPS <= f.gap <= p.g_max + EPS || return "swimmer $i gap $(f.gap) out of range"
        i != AGENT && pausing(f) && return "field swimmer $i sighting while following"
    end
    return nothing
end
