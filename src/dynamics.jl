# dynamics.jl
#
# Motion of a single swimmer over one short time segment:
#   - swimming: forward speed v along heading ψ, plus current c
#   - pausing to sight: current only
#   - heading drift (bias + noise) and position noise, scaled by segment length
#   - applying the agent's turn δ (clamped to δmax)
# Plus geometry helpers used everywhere: bearing and distance to the buoy, and
# whether a path segment passes within the finish radius.
