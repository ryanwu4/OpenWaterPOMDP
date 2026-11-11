# types.jl
#
# Plain data types shared by the whole package. All are immutable and isbits so states
# are cheap to copy.
#
# Swimmers live in one fixed-size vector: index AGENT (= 1) is the agent, 2..K+1 are the
# field. Agent-specific rules are index checks, not a separate code path.

const AGENT = 1

"""
One swimmer. Dynamic fields change every step; per-race constants are sampled once in
the initial state (kept here, not in the params, so beliefs can be uncertain about them).

Timers count down in seconds; `Inf` means "not scheduled".
"""
struct Swimmer
    # dynamic
    x::Float64
    y::Float64
    ψ::Float64              # heading, rad, relative to the start–buoy line
    leader::Int             # index being followed, 0 = solo
    gap::Float64            # along-track gap behind the leader (when following)
    pause::Float64          # remaining sight pause; > 0 means pausing
    sight_timer::Float64    # time until the next sight pause starts
    draft_timer::Float64    # remaining draft time (field swimmers)
    cooldown::Float64       # time before a field swimmer may draft again
    finished::Bool
    # per-race constants
    v::Float64              # pace, m/s
    b::Float64              # heading bias, rad/s
    sight_period::Float64   # seconds between field sights (Inf for the agent)
    draft_duration::Float64 # field draft length (Inf for the agent)
end

"""
    Swimmer(; x, y, ψ, v, ...)

Keyword constructor for building scenarios by hand. Defaults: solo, swimming, no timers
scheduled, no bias.
"""
Swimmer(; x = 0.0, y = 0.0, ψ = 0.0, leader = 0, gap = 0.0, pause = 0.0, sight_timer = Inf,
        draft_timer = Inf, cooldown = 0.0, finished = false, v = 1.3, b = 0.0,
        sight_period = Inf, draft_duration = Inf) =
    Swimmer(x, y, ψ, leader, gap, pause, sight_timer, draft_timer, cooldown, finished,
            v, b, sight_period, draft_duration)

"""
Full world state. N = K + 1 swimmers. `t` is elapsed race time, `c` the current.
"""
struct SwimState{N}
    t::Float64
    c::SVector{2,Float64}
    swimmers::SVector{N,Swimmer}
end

"""
Agent action: sight this cycle, turn δ (rad, clamped to ±δmax, ignored while
following), and follow (true) or swim solo (false).
"""
struct SwimAction
    sight::Bool
    turn::Float64
    follow::Bool
end

SwimAction(; sight = false, turn = 0.0, follow = false) = SwimAction(sight, turn, follow)

agent(s::SwimState) = s.swimmers[AGENT]
following(sw::Swimmer) = sw.leader != 0
pausing(sw::Swimmer) = sw.pause > 1e-9   # matches EPS in drafting.jl
