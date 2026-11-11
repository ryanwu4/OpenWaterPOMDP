# OpenWaterPOMDP.jl
# Package entry point: imports, includes, exports. Include order matters (types before users of them).
module OpenWaterPOMDP

using Random
using StaticArrays
using Parameters: @with_kw
using ConstructionBase: setproperties
using POMDPs
using POMDPTools: ImplicitDistribution

export SwimParams, Swimmer, SwimState, SwimAction, AGENT
export SwimMDP, SwimActionSpace
export FieldRulePolicy, reset!
export step_race, advance!, sample_initial_state, check_invariants
export agent, following, pausing, bearing_to_buoy, dist_to_buoy

include("params.jl")
include("types.jl")
include("dynamics.jl")
include("drafting.jl")
include("field.jl")
include("advance.jl")
include("pomdp.jl")
include("observation.jl")
include("belief.jl")
include("policies/baselines.jl")
include("policies/oracle.jl")
include("policies/planners.jl")
include("metrics.jl")

end # module
