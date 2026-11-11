# params.jl
#
# The model spec: every constant and every per-race distribution, as plain numbers.
# Angles are in radians, times in seconds, distances in meters.
# A different condition is a different instance, e.g. `SwimParams(K = 5, current_speed = (0.2, 0.4))`.
# Per-race random quantities (current, paces, biases, start positions, ...) are sampled
# from these into the initial state; see `sample_initial_state`.
# Default values are proposals; see docs/simulator_v1.md §9.

@with_kw struct SwimParams
    # course
    L::Float64                  = 1500.0           # buoy at (L, 0)
    finish_radius::Float64      = 3.0
    start_halfwidth::Float64    = 20.0             # start y ~ U(-w, w) at x = 0

    # agent
    v_agent::Float64            = 1.3              # m/s

    # field
    K::Int                      = 10               # number of field swimmers
    field_pace_mean::Float64    = 1.3
    field_pace_std::Float64     = 0.1
    field_pace_range::Tuple{Float64,Float64} = (1.0, 1.7)   # clip
    sight_strokes::Tuple{Int,Int} = (6, 12)        # N_k ~ U{lo, ..., hi}
    draft_duration::Tuple{Float64,Float64} = (60.0, 300.0)  # per swimmer ~ U(lo, hi)
    draft_cooldown::Float64     = 60.0
    σ_aim::Float64              = deg2rad(5.0)     # field re-aim error

    # timing
    τ0::Float64                 = 2.5              # one decision cycle without a sight
    τκ::Float64                 = 1.0              # extra time a sight costs
    strokes_per_cycle::Int      = 3                # converts N_k strokes into seconds

    # heading and motion noise (rates per second of swimming)
    δmax::Float64               = deg2rad(30.0)    # max turn per cycle
    σψ0::Float64                = deg2rad(5.0)     # initial aim error
    σψ::Float64                 = deg2rad(1.0)     # heading noise, rad/√s
    σ_bias::Float64             = deg2rad(0.15)    # per-swimmer bias ~ N(0, σ_bias²), rad/s
    σw::Float64                 = 0.2              # position noise, m/√s

    # current: speed ~ U(lo, hi), direction ~ U(lo, hi), mirrored across the course line
    # with probability 1/2 if current_random_side (default: within ±45° of cross-course)
    current_speed::Tuple{Float64,Float64} = (0.0, 0.3)
    current_dir::Tuple{Float64,Float64}   = (π / 4, 3π / 4)
    current_random_side::Bool             = true

    # drafting
    β::Float64                  = 0.20             # follower may swim at (1+β)v
    g_nom::Float64              = 0.5              # nominal gap behind the leader
    g_max::Float64              = 2.0              # draft lost beyond this gap
    w_lat::Float64              = 0.5              # zone half-width

    # simulation
    h_max::Float64              = 0.5              # longest segment inside advance
    t_cap::Float64              = 3 * L / v_agent  # time cap (recomputed from L, v_agent)
    discount::Float64           = 1.0
end
