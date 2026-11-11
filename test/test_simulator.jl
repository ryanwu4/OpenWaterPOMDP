using Random, StaticArrays, POMDPs

const QUIET = (σψ = 0.0, σψ0 = 0.0, σ_bias = 0.0, σw = 0.0, current_speed = (0.0, 0.0))

state(swimmers...) = SwimState(0.0, SVector(0.0, 0.0), SVector(swimmers...))

# Run full races, checking invariants after every step.
function fuzz(p, n, rng; random_actions)
    m = SwimMDP(p)
    for _ in 1:n
        s = rand(rng, initialstate(m)); k = 0
        while !isterminal(m, s)
            a = if random_actions
                rand(rng, actions(m))
            else
                ag = agent(s)
                SwimAction(sight = k % 4 == 0, follow = true,
                           turn = OpenWaterPOMDP.wrap(bearing_to_buoy(p, ag.x, ag.y) - ag.ψ))
            end
            sp, r = step_race(p, s, a, rng)
            err = check_invariants(p, sp)
            err === nothing || return "step $k: $err"
            sp.t >= s.t || return "time went backwards"
            timeout = !agent(sp).finished && isterminal(m, sp)
            (timeout || r ≈ -(sp.t - s.t)) || return "reward $r != -elapsed"
            for i in eachindex(s.swimmers)
                s.swimmers[i].finished && sp.swimmers[i] != s.swimmers[i] && return "finished swimmer $i changed"
            end
            s = sp; k += 1
        end
    end
    return nothing
end

@testset "invariants over full races" begin
    rng = MersenneTwister(1)
    @test fuzz(SwimParams(), 20, rng; random_actions = false) === nothing
    @test fuzz(SwimParams(), 5, rng; random_actions = true) === nothing
    crowded = SwimParams(K = 6, start_halfwidth = 3.0)          # lots of drafting
    @test fuzz(crowded, 30, rng; random_actions = false) === nothing
    @test fuzz(crowded, 10, rng; random_actions = true) === nothing
end

@testset "noiseless straight swim" begin
    p = SwimParams(; K = 0, start_halfwidth = 0.0, QUIET...)
    m = SwimMDP(p); rng = MersenneTwister(2)
    s = rand(rng, initialstate(m))
    while !isterminal(m, s)
        s, _ = step_race(p, s, SwimAction(), rng)
    end
    @test agent(s).finished
    @test s.t ≈ (p.L - p.finish_radius) / p.v_agent
end

@testset "sighting while following opens a gap of v_lead·τκ" begin
    p = SwimParams(; K = 1, QUIET...)
    lead = Swimmer(x = 100.0, v = 1.3, sight_timer = 1e6, sight_period = 1e6)
    ag = Swimmer(x = 100.0 - p.g_nom, v = 1.3, leader = 2, gap = p.g_nom)
    rng = MersenneTwister(3)
    sp, _ = step_race(p, state(ag, lead), SwimAction(sight = true, follow = true), rng)
    @test following(agent(sp))
    @test agent(sp).gap ≈ p.g_nom + lead.v * p.τκ
    # without sighting, (1+β)v ≥ v_lead holds the nominal gap
    sp, _ = step_race(p, state(ag, lead), SwimAction(follow = true), rng)
    @test agent(sp).gap ≈ p.g_nom
end

@testset "field swimmer drops its follower before drafting" begin
    p = SwimParams(; K = 2, QUIET...)
    fast = Swimmer(x = 100.0, v = 1.6, sight_timer = 1e6, sight_period = 1e6)
    mid = Swimmer(x = 99.0, v = 1.3, sight_timer = 1e6, sight_period = 1e6, draft_duration = 100.0)
    ag = Swimmer(x = 98.5, v = 1.3, leader = 2, gap = 0.5)        # agent follows `mid`
    s = state(ag, mid, fast)
    @test check_invariants(p, s) === nothing
    sp, _ = step_race(p, s, SwimAction(follow = true), MersenneTwister(4))
    @test sp.swimmers[2].leader == 3          # mid now follows fast
    @test !following(agent(sp))               # agent was released and can't follow mid
    @test check_invariants(p, sp) === nothing
end

@testset "params and POMDPs wrapper" begin
    @test SwimParams(L = 500.0).t_cap ≈ 3 * 500 / 1.3
    @test SwimParams(v_agent = 1.0).t_cap ≈ 3 * 1500 / 1.0
    m = SwimMDP()
    s = rand(MersenneTwister(5), initialstate(m))
    sp, r = @gen(:sp, :r)(m, s, SwimAction(turn = 0.1), MersenneTwister(5))
    @test sp.t ≈ m.params.τ0 && r ≈ -m.params.τ0
end
