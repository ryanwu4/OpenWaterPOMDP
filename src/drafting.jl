# drafting.jl
#
# Interactions between swimmers:
#   - whether a swimmer is in another's draft zone
#   - choosing a leader when more than one is available
#   - motion while following: track the leader's path at speed min(v_lead, (1+β)v),
#     inheriting the leader's navigation errors
#   - when following ends (leader too far ahead, swimmer chooses solo, time limit for the field)
