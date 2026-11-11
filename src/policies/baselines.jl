# baselines.jl
#
# Rule-based policies for the agent, as POMDPs.jl Policy types:
#   - sighting: sight every N strokes and re-aim at the buoy
#   - drafting: never, always, or for a fixed duration
# Parameterized (N, duration) so they can be tuned per condition. Also used as
# rollout policies by the planners.

"""
    FieldRulePolicy(p; sight_cycles = 3, draft_duration = 180.0, cooldown = p.draft_cooldown)

The agent following the field's rule:
- sight every `sight_cycles` cycles, then turn toward the buoy bearing on the next cycle
  ("sight now, turn next")
- draft a faster swimmer it comes into contact with for `draft_duration` seconds, then
  swim solo for `cooldown` seconds before drafting again
- no sighting while drafting; sight right after a draft ends

For now it acts on the true state (`SwimMDP`): a sight reveals the exact bearing, and it
knows other swimmers' paces. Once the observation model exists it should act on
observations instead.

Stateful (it remembers its sight schedule and draft timing); use a fresh instance, or
`reset!`, for each race.
"""
mutable struct FieldRulePolicy <: Policy
    p::SwimParams
    sight_cycles::Int
    draft_duration::Float64
    cooldown::Float64
    # memory
    cycles_since_sight::Int
    just_sighted::Bool
    was_following::Bool
    draft_started::Float64
    cooldown_until::Float64
end

function FieldRulePolicy(p::SwimParams; sight_cycles = 3, draft_duration = 180.0,
                         cooldown = p.draft_cooldown)
    return FieldRulePolicy(p, sight_cycles, draft_duration, cooldown, sight_cycles, false,
                           false, -Inf, -Inf)
end

function reset!(pol::FieldRulePolicy)
    pol.cycles_since_sight = pol.sight_cycles
    pol.just_sighted = pol.was_following = false
    pol.draft_started = pol.cooldown_until = -Inf
    return pol
end

"""
Whether the leader the simulator would assign on "follow" (the nearest eligible one) is
faster than the agent.
"""
function faster_leader_available(p::SwimParams, s::SwimState)
    sw = s.swimmers
    fol = followers(sw)
    best_g, best_j = Inf, 0
    for j in eachindex(sw)
        (j != AGENT && can_lead(sw, j) && fol[j] == 0) || continue
        g = zone_gap(p, sw[AGENT], sw[j])
        g !== nothing && g < best_g && ((best_g, best_j) = (g, j))
    end
    return best_j != 0 && sw[best_j].v > sw[AGENT].v
end

function POMDPs.action(pol::FieldRulePolicy, s::SwimState)
    p, ag = pol.p, agent(s)

    # draft bookkeeping: notice drafts starting and ending
    if pol.was_following && !following(ag)
        pol.cooldown_until = s.t + pol.cooldown
        pol.cycles_since_sight = pol.sight_cycles          # sight right away
    elseif !pol.was_following && following(ag)
        pol.draft_started = s.t
    end
    pol.was_following = following(ag)

    follow = following(ag) ? s.t - pol.draft_started < pol.draft_duration :
             s.t >= pol.cooldown_until && faster_leader_available(p, s)
    staying = following(ag) && follow

    turn = pol.just_sighted && !staying ? wrap(bearing_to_buoy(p, ag.x, ag.y) - ag.ψ) : 0.0
    sight = !staying && pol.cycles_since_sight >= pol.sight_cycles

    pol.just_sighted = sight
    pol.cycles_since_sight = sight ? 1 : pol.cycles_since_sight + 1
    return SwimAction(sight, turn, follow)
end
