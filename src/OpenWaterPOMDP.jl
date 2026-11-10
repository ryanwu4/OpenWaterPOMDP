# OpenWaterPOMDP.jl
# Package entry point: imports, includes, exports. Include order matters (types before users of them).
module OpenWaterPOMDP

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
