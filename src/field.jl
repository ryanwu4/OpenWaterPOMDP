# field.jl
#
# The rule-based behavior of the K non-agent swimmers, applied inside every transition:
# sight every N_k strokes and re-aim at the buoy, draft any faster swimmer they contact
# for a fixed time. Written as an interchangeable rule so other field behaviors can
# be substituted later.
