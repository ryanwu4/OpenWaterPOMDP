# demo.jl — animate one race with the agent following the field's rule.
#
#   julia --project=analysis analysis/demo.jl [seed] [output.gif]
#
# Strong cross-current (0.3 m/s toward +y) so drift is visible. Left panel: the whole
# course with everyone's trail. Right panel: a 30 m window following the agent, showing
# swimmers as stick figures (hollow head = sighting), drafting pairs (green rings) and the
# buoy (or an arrow pointing to it when it's out of view). The close-up uses coordinates
# relative to the agent so its axes stay fixed from frame to frame.

ENV["GKSwstype"] = "100"     # render off-screen: no GR window pops up
using OpenWaterPOMDP, POMDPs, POMDPTools, Plots, Random, Printf

const SEED = parse(Int, get(ARGS, 1, "10"))
const OUT = get(ARGS, 2, joinpath(@__DIR__, "..", "results", "demo.gif"))

p = SwimParams(current_speed = (0.3, 0.3), current_dir = (π / 2, π / 2), current_random_side = false)
m = SwimMDP(p)
rng = MersenneTwister(SEED)
s0 = rand(rng, initialstate(m))
hist = simulate(HistoryRecorder(rng = rng), m, FieldRulePolicy(p), s0)

states = [s0; collect(state_hist(hist))[2:end]]
acts = collect(action_hist(hist))
n, N = length(states), length(s0.swimmers)
X = [states[k].swimmers[i].x for k in 1:n, i in 1:N]
Y = [states[k].swimmers[i].y for k in 1:n, i in 1:N]

sf = states[end]
drafted = count(k -> following(agent(states[k])), 1:n)
@printf("seed %d: agent %s at t = %.0f s, %d sights, following %.0f%% of steps\n", SEED,
        agent(sf).finished ? "finished" : "timed out", sf.t, count(a -> a.sight, acts), 100drafted / n)

const AGENT_COLOR = :crimson
const FIELD_COLOR = :steelblue
ylo, yhi = extrema(Y)
pad = 0.1 * max(yhi - ylo, 50)

function course_panel(k)
    plt = plot(xlims = (-30, p.L + 50), ylims = (ylo - pad, yhi + pad), legend = false,
               xlabel = "x (m)", ylabel = "y (m)", title = "Course", grid = false)
    for i in N:-1:1
        plot!(plt, X[1:k, i], Y[1:k, i], color = i == AGENT ? AGENT_COLOR : FIELD_COLOR,
              lw = i == AGENT ? 2.5 : 1, alpha = i == AGENT ? 1 : 0.5)
    end
    scatter!(plt, X[k, 2:end], Y[k, 2:end], color = FIELD_COLOR, ms = 3, msw = 0)
    scatter!(plt, [X[k, AGENT]], [Y[k, AGENT]], color = AGENT_COLOR, ms = 5, msw = 0)
    scatter!(plt, [p.L], [0.0], color = :orange, ms = 9, marker = :circle)
    c = s0.c / hypot(s0.c...) * 0.12 * (yhi - ylo + 2pad)   # current arrow, top-right corner
    ox, oy = p.L - 150, yhi + pad / 2 - max(c[2], 0)
    plot!(plt, [ox, ox + c[1]], [oy, oy + c[2]], arrow = true, color = :gray30, lw = 2)
    annotate!(plt, ox + 15, oy + c[2] / 2, text(@sprintf("current %.2f m/s", hypot(s0.c...)), 8, :left, :gray30))
    return plt
end

const BODY = 2.0          # drawn body length, m (a little larger than life so it reads)

"""
Top-down stick figure with its head at (x, y), pointing along heading ψ.
Swimming: freestyle arms, one reaching forward and one pulling back, alternating with
`phase`. Sighting (`paused`): head up (hollow) and both arms sculling out to the sides.
"""
function stick_figure!(plt, x, y, ψ, paused, color; phase = 0.0, scale = BODY, lw = 2)
    u = (cos(ψ), sin(ψ))                       # forward
    v = (-u[2], u[1])                          # to the swimmer's left
    pt(f, l) = (x + scale * (f * u[1] + l * v[1]), y + scale * (f * u[2] + l * v[2]))
    segs = Tuple{Float64,Float64}[]
    seg!(a, b) = append!(segs, [pt(a...), pt(b...), (NaN, NaN)])
    seg!((-0.12, 0.0), (-0.65, 0.0))           # body: neck to hips
    seg!((-0.20, 0.16), (-0.20, -0.16))        # shoulders
    if paused
        seg!((-0.20, 0.16), (0.05, 0.42))      # arms sculling out to the sides
        seg!((-0.20, -0.16), (0.05, -0.42))
    else
        reach = 0.38 * cos(phase)              # freestyle: arms alternate
        seg!((-0.20, 0.16), (-0.20 + reach, 0.20))
        seg!((-0.20, -0.16), (-0.20 - reach, -0.20))
    end
    kick = 0.04 * sin(3phase)                  # flutter kick
    seg!((-0.65, 0.07), (-1.0, 0.10 + kick))
    seg!((-0.65, -0.07), (-1.0, -0.10 - kick))
    plot!(plt, first.(segs), last.(segs), color = color, lw = lw)
    θ = range(0, 2π, length = 16)               # head
    r = 0.1 * scale
    plot!(plt, Shape(x .+ r .* cos.(θ), y .+ r .* sin.(θ)),
          color = paused ? :white : color, linecolor = color, lw = lw)
    return plt
end

"""
Where to draw each swimmer in the close-up. A real draft gap (≥ 0.5 m) is shorter than a
drawn body, so followers are drawn behind their leader's feet: one body length plus the
actual gap behind the leader's drawn position. Everyone else is drawn where they are.
"""
function drawn_positions(s)
    pos = Dict{Int,Tuple{Float64,Float64}}()
    function place(i)
        haskey(pos, i) && return pos[i]
        sw = s.swimmers[i]
        if following(sw)
            l = s.swimmers[sw.leader]
            lx, ly = place(sw.leader)
            d = BODY + sw.gap
            pos[i] = (lx - d * cos(l.ψ), ly - d * sin(l.ψ))
        else
            pos[i] = (sw.x, sw.y)
        end
    end
    foreach(place, eachindex(s.swimmers))
    return pos
end

function ring!(plt, x, y, r)
    θ = range(0, 2π, length = 32)
    plot!(plt, x .+ r .* cos.(θ), y .+ r .* sin.(θ), color = :green3, lw = 2.5)
end

"""Buoy in the close-up: drawn to scale when in view, otherwise an arrow at the edge."""
function buoy_marker!(plt, bx, by, w)
    if max(abs(bx), abs(by)) < w
        θ = range(0, 2π, length = 32)
        r = p.finish_radius
        plot!(plt, Shape(bx .+ r .* cos.(θ), by .+ r .* sin.(θ)), color = :orange,
              linecolor = :darkorange3, fillalpha = 0.35, lw = 2)
        scatter!(plt, [bx], [by], color = :orange, msc = :darkorange3, ms = 8)
    else
        d = (bx, by) ./ hypot(bx, by)
        e = d .* (0.88w / max(abs(d[1]), abs(d[2])))   # point on the edge of the view
        plot!(plt, [e[1] - 3d[1], e[1]], [e[2] - 3d[2], e[2]], arrow = arrow(:closed, :head, 1.2, 1.2),
              color = :darkorange2, lw = 3)
        annotate!(plt, e[1] - 5.5d[1], e[2] - 5.5d[2],
                  text(@sprintf("buoy %.0f m", hypot(bx, by)), 8, :darkorange3))
    end
end

function closeup_panel(k)
    s = states[k]
    ax, ay = X[k, AGENT], Y[k, AGENT]
    w = 15.0
    plt = plot(xlims = (-w, w), ylims = (-w, w), aspect_ratio = :equal, legend = false,
               grid = false, xticks = -15:5:15, yticks = -15:5:15,
               xlabel = "x from agent (m)", ylabel = "y from agent (m)")
    lo = max(1, k - 15)
    for i in 1:N                                       # short trails
        plot!(plt, X[lo:k, i] .- ax, Y[lo:k, i] .- ay, color = i == AGENT ? AGENT_COLOR : FIELD_COLOR,
              alpha = 0.35, lw = 1)
    end
    buoy_marker!(plt, p.L - ax, -ay, w)
    pos = drawn_positions(s)
    for (i, sw) in enumerate(s.swimmers)
        sw.finished && continue
        x, y = pos[i] .- (ax, ay)
        if following(sw)                               # drafting: green ring around the pair
            l = s.swimmers[sw.leader]
            lx, ly = pos[sw.leader] .- (ax, ay)
            mx, my = (x + lx) / 2 - 0.5BODY * cos(l.ψ), (y + ly) / 2 - 0.5BODY * sin(l.ψ)
            ring!(plt, mx, my, 0.5 * hypot(lx - x, ly - y) + 0.9BODY)
        end
        stick_figure!(plt, x, y, sw.ψ, pausing(sw), i == AGENT ? AGENT_COLOR : FIELD_COLOR;
                      phase = 2π * s.t / p.τ0 + i, lw = i == AGENT ? 2.5 : 1.8)
    end
    a = agent(s)
    status = following(a) ? "drafting #$(a.leader)" : "solo"
    k > 1 && acts[k-1].sight && (status *= ", sighted")
    title!(plt, @sprintf("t = %.0f s   %.0f m to go   %s", s.t, dist_to_buoy(p, a.x, a.y), status),
           titlefontsize = 11, titlelocation = :left)
    return plt
end

anim = @animate for k in 1:n
    plot(course_panel(k), closeup_panel(k), layout = grid(1, 2, widths = [0.6, 0.4]),
         size = (1300, 560), left_margin = 4Plots.mm, bottom_margin = 4Plots.mm)
end
mkpath(dirname(OUT))
gif(anim, OUT, fps = 20)
