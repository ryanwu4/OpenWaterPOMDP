# belief.jl
#
# The agent's belief over the hidden state (its own position and heading, the current,
# and the field), built on ParticleFilters.jl:
#   - initial belief from the params' priors
#   - updater used both in simulation and by the planners
#   - protection against particle depletion during long stretches without sighting
#     (e.g. roughening / noise injection)
