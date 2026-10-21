# =============================================================================
# DEEulerSolution: Wrapper module for 1st-order Forward Euler predictor along the
# Davidenko differential equation followed by Newton correction (`:euler` mode).
# =============================================================================
module DEEulerSolution
import ..DESolution
export aim_cue!
@inline @fastmath aim_cue!(data, sid::Int32) = (DESolution.set_predictor_mode!(:euler); DESolution.aim_cue!(data, sid))
end

