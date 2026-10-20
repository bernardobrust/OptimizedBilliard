# =============================================================================
# DEConstantSolution: Wrapper module for 0th-order (Constant / finite-difference)
# predictor-corrector continuation (`:zero_order` mode in DESolution).
# =============================================================================
module DEConstantSolution
import ..DESolution
export aim_cue!
@inline aim_cue!(data, sid::Int32) = (DESolution.set_predictor_mode!(:zero_order); DESolution.aim_cue!(data, sid))
end

