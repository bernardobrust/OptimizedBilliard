module DEConstantSolution
import ..DESolution
export aim_cue!
@inline aim_cue!(data, sid::Int32) = (DESolution.set_predictor_mode!(:zero_order); DESolution.aim_cue!(data, sid))
end
