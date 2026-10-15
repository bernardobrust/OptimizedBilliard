module DEEulerSolution
import ..DESolution
export aim_cue!
@inline aim_cue!(data, sid::Int32) = (DESolution.set_predictor_mode!(:euler); DESolution.aim_cue!(data, sid))
end
