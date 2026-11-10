# params.jl
#
# The model spec. A single immutable parameter struct with keyword defaults holding every
# constant in the model, e.g.:
#   - course: buoy distance L, finish radius, time cap
#   - swimmer: pace distribution, distance per cycle ℓ, cycle time τ0, sight cost τκ
#   - noise: heading noise σψ, position noise w, bearing noise σo, field-position noise
#   - current: prior over current strength/direction (and spatial/temporal structure, if any)
#   - drafting: draft-zone geometry, boost β, field drafting duration
#   - field: number of swimmers K, sight interval N_k
#   - actions: turn increment Δ
#   - discount
#
# Every other file reads constants from this struct; nothing is hard-coded elsewhere.
# The POMDP type in pomdp.jl stores an instance, so a different condition is just a
# different instance (override fields by keyword).
#
# Quantities that are random per race (actual current, paces, start positions) are not
# stored here. Their *distributions* are, and they are sampled into the initial state.
