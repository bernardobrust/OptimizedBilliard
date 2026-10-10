# x, p and c are vectors implicitly, t is a scalar time parameter implicit in all functions wich determines the values of c and p
# delta_t is hyperparameter

# == 0-order Predictor-corrector model (a.k.a constant prediction, a.k.a Newton tracker) ==


# jacobian (x) => Jacobian aplied on x
# f(x) = line-circle error system

# f(x) = eps_1 { n_transpose * (x - p) }
#      eps_2 { (x - c)_transpose * (x - c) }

# Use eps_1 and eps_2 as the equations for the jacobian
# Solve the system for the new eps_1 and eps_2 from either the simple solution
# or from the previous frame. Dicretize the dt such that the number of Newton steps
# required for covergion s.t it's limited to 2 steps.
# This way we can ajust the convergence point to the Newton result.

# We'll need J as functions, J_inv for the analytical inverse of a 2x2
# Newton is defined as:
# x_n+1 = X_n - J_inv * f(x_n)
# Or we can use:
# dx = - J_inv * f(x_n)

# The 't' will be new, so f(x_n) may not converge to 0


# == 1-order Predictor-corrector model (using a 4th order Runge Kutta) ==


@TODO after 0-order


# == Investigations ==

# Optimization conserns:
# 1. dt value may be dependent of the speed of the ball
# 2. collisions will cause the predictor to fail
# 2.1 we may want to signal a colision to the predictor
# 3. use the simple solution in cases the Newton does not converge
# 4. reasonable tolerance to use
# 5. benchmark againt simple solution

# Correctude:
# 1. maybe randomize the speeds every so often to check if Newton works well
# 1.1 right now speed is constant when not coliding
# 2. if the intersection was on a line perpendicular to the LOS and crossing the center
# of the circle Newton may converge to the wrong root