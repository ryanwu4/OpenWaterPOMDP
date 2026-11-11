# pomdp.jl
#
# POMDPs.jl problem definitions on top of the simulator core (step_race, advance!).
#   - sample_initial_state: draws the per-race quantities (current, paces, biases, start
#     positions, field sight schedules and draft durations) from the params
#   - SwimMDP: the fully observed problem (used by the oracle, and to test the simulator):
#     gen, initialstate, isterminal, discount, actions
# The POMDP version (SwimPOMDP) reuses the same transition and adds the observation
# model from observation.jl.

"""
    sample_initial_state(p, Val(N), rng)

A random race start with N = K + 1 swimmers: everyone at x = 0, spread in y, aimed at
the buoy with a small error. The agent is index AGENT.
"""
function sample_initial_state(p::SwimParams, ::Val{N}, rng::AbstractRNG) where {N}
    N == p.K + 1 || throw(ArgumentError("N = $N but params have K = $(p.K)"))
    θ = p.current_dir[1] + (p.current_dir[2] - p.current_dir[1]) * rand(rng)
    p.current_random_side && rand(rng, Bool) && (θ = -θ)
    speed = p.current_speed[1] + (p.current_speed[2] - p.current_speed[1]) * rand(rng)
    c = SVector(speed * cos(θ), speed * sin(θ))
    swimmers = SVector{N,Swimmer}(ntuple(i -> sample_swimmer(p, i, rng), Val(N)))
    return SwimState{N}(0.0, c, swimmers)
end

function sample_swimmer(p::SwimParams, i::Int, rng::AbstractRNG)
    y = p.start_halfwidth * (2rand(rng) - 1)
    ψ = wrap(bearing_to_buoy(p, 0.0, y) + p.σψ0 * randn(rng))
    b = p.σ_bias * randn(rng)
    if i == AGENT
        return Swimmer(0.0, y, ψ, 0, 0.0, 0.0, Inf, Inf, 0.0, false, p.v_agent, b, Inf, Inf)
    end
    v = clamp(p.field_pace_mean + p.field_pace_std * randn(rng), p.field_pace_range...)
    n_strokes = rand(rng, p.sight_strokes[1]:p.sight_strokes[2])
    period = n_strokes / p.strokes_per_cycle * p.τ0
    duration = p.draft_duration[1] + (p.draft_duration[2] - p.draft_duration[1]) * rand(rng)
    return Swimmer(0.0, y, ψ, 0, 0.0, 0.0, period * rand(rng), Inf, 0.0, false,
                   v, b, period, duration)
end

"""
    SwimMDP(params = SwimParams())

Fully observed race: the state includes the current and every swimmer's constants.
"""
struct SwimMDP{N} <: MDP{SwimState{N},SwimAction}
    params::SwimParams
end
SwimMDP(p::SwimParams = SwimParams()) = SwimMDP{p.K + 1}(p)

params(m::SwimMDP) = m.params

function POMDPs.gen(m::SwimMDP, s::SwimState, a::SwimAction, rng::AbstractRNG)
    sp, r = step_race(m.params, s, a, rng)
    return (sp = sp, r = r)
end

POMDPs.initialstate(m::SwimMDP{N}) where {N} =
    ImplicitDistribution(rng -> sample_initial_state(m.params, Val(N), rng))
POMDPs.isterminal(m::SwimMDP, s::SwimState) = isterminal_state(m.params, s)
POMDPs.discount(m::SwimMDP) = m.params.discount

"""
The hybrid action space: sight ∈ {0, 1}, turn ∈ [−δmax, δmax], follow ∈ {0, 1}.
`rand` samples uniformly; planners should use their own action samplers.
"""
struct SwimActionSpace
    δmax::Float64
end
POMDPs.actions(m::SwimMDP) = SwimActionSpace(m.params.δmax)

Base.rand(rng::AbstractRNG, sp::Random.SamplerTrivial{SwimActionSpace}) =
    SwimAction(rand(rng, Bool), sp[].δmax * (2rand(rng) - 1), rand(rng, Bool))
