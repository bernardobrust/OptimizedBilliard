# =============================================================================
# DERK4Solution: Wrapper module for 4th-order Runge-Kutta (RK4) predictor along
# the Davidenko differential equation followed by Newton correction (`:rk4` mode).
# =============================================================================
module DERK4Solution
import ..DESolution
export aim_cue!
@inline aim_cue!(data, sid::Int32) = (DESolution.set_predictor_mode!(:rk4); DESolution.aim_cue!(data, sid))
end

