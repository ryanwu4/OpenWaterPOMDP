# dynamics.jl
#
# Motion of a single solo swimmer over one short segment, plus geometry helpers.
# A swimmer is either swimming (v along ψ, plus current; heading drifts with bias and
# noise) or pausing to sight (current only, no heading drift). Both get position noise.
# Noise is scaled by segment length, so splitting a step into segments is exact.

wrap(ψ) = rem2pi(ψ, RoundNearest)

buoy(p::SwimParams) = SVector(p.L, 0.0)
bearing_to_buoy(p::SwimParams, x, y) = atan(-y, p.L - x)
dist_to_buoy(p::SwimParams, x, y) = hypot(p.L - x, y)

clamp_turn(p::SwimParams, δ) = clamp(δ, -p.δmax, p.δmax)

"""
    move_solo(p, sw, c, h, rng) -> (swimmer, through_water_distance)

Advance a solo swimmer by h seconds. The displacement uses the heading at the start of
the segment; heading drift is applied at the end (Euler step).
"""
function move_solo(p::SwimParams, sw::Swimmer, c::SVector{2,Float64}, h::Float64, rng::AbstractRNG)
    swimming = !pausing(sw)
    a = swimming ? sw.v * h : 0.0
    sq = sqrt(h)
    x = sw.x + a * cos(sw.ψ) + c[1] * h + p.σw * sq * randn(rng)
    y = sw.y + a * sin(sw.ψ) + c[2] * h + p.σw * sq * randn(rng)
    ψ = swimming ? wrap(sw.ψ + sw.b * h + p.σψ * sq * randn(rng)) : sw.ψ
    return setproperties(sw, (x = x, y = y, ψ = ψ)), a
end

"""
    finish_fraction(p, x0, y0, x1, y1)

Fraction s ∈ [0, 1] along the segment (x0, y0) → (x1, y1) at which a swimmer first comes
within the finish radius of the buoy, or -1.0 if it doesn't.
"""
function finish_fraction(p::SwimParams, x0, y0, x1, y1)
    r = p.finish_radius
    fx, fy = x0 - p.L, y0          # start relative to the buoy
    dx, dy = x1 - x0, y1 - y0
    c0 = fx^2 + fy^2 - r^2
    c0 <= 0 && return 0.0
    a = dx^2 + dy^2
    a == 0 && return -1.0
    b = 2 * (fx * dx + fy * dy)
    disc = b^2 - 4a * c0
    disc < 0 && return -1.0
    s = (-b - sqrt(disc)) / (2a)
    return 0.0 <= s <= 1.0 ? s : -1.0
end
