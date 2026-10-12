module DESolution

# =============================================================================
# Differential Equations / Predictor-Corrector solution for cue aiming in billiards.
#
# MATHEMATICAL FORMULATION:
# =============================================================================
# 1. Coordinate Definitions:
#    - x = (x, y)ᵀ: 2D intersection / collision coordinate.
#    - p = (px, py)ᵀ: Cue pivot / ray origin (data.cue_x, data.cue_y).
#    - u = (ux, uy)ᵀ: Normalized unit direction vector along Line of Sight (LOS).
#    - n = (nx, ny)ᵀ = (uy, -ux)ᵀ: Unit normal vector orthogonal to LOS (nᵀ u = 0).
#    - c = (cx, cy)ᵀ: Ball center (data.ball_x[j], data.ball_y[j]).
#    - rad = R: Ball radius.
#
# 2. Nonlinear Error System f(x) = [ε₁, ε₂]ᵀ:
#    - ε₁: Line constraint (distance to LOS line):
#        f₁(x) = nᵀ (x - p) = uy * (x - px) - ux * (y - py) = 0
#    - ε₂: Circle constraint (distance to circle perimeter):
#        f₂(x) = (x - c)ᵀ (x - c) - rad² = (x - cx)² + (y - cy)² - rad² = 0
#
# 3. Analytical Jacobian Matrix J(x) and Inverse J⁻¹:
#    - J(x) = [ ∂f₁/∂x   ∂f₁/∂y ] = [      uy             -ux     ]
#             [ ∂f₂/∂x   ∂f₂/∂y ]   [ 2 * (x - cx)   2 * (y - cy) ]
#    - Determinant:
#        det(J) = 2 * (uy * (y - cy) + ux * (x - cx)) = 2 * uᵀ (x - c)
#        Note: det(J) = 0 only when x - c is perpendicular to u (line tangent to circle).
#    - Analytical Inverse:
#        J⁻¹ = (1 / det(J)) * [ 2 * (y - cy)    ux ]
#                             [ -2 * (x - cx)   uy ]
#
# 4. Newton-Raphson Corrector:
#    - Given an estimate X_n:
#        Δx = J(X_n)⁻¹ * f(X_n)
#        X_{n+1} = X_n - Δx
#      Because the line constraint is linear, this iteration contracts quadratically
#      to the intersection root in 1-2 steps.
#
# 5. Continuous Path Tracking via Davidenko Differential Equation:
#    - Differentiating f(x(t), t) = 0 with respect to parameter / physical time t:
#        J(x(t)) * (dx/dt) + ∂f/∂t = 0
#        => dx/dt = - J(x(t))⁻¹ * (∂f/∂t)
#      where ∂f/∂t is computed holding x constant:
#        ∂f₁/∂t = (dn/dt)ᵀ (x - p) - nᵀ (dp/dt)
#        ∂f₂/∂t = -2 * (x - c)ᵀ (dc/dt)
#
# 6. Predictor-Corrector Architecture:
#    - 0-order Predictor (Newton tracker / constant prediction):
#        X_pred = X_prev
#        X_corr = newton_step(X_pred, ...)
#    - 1-order Predictor (4th-order Runge-Kutta integration of Davidenko DE):
#        X_pred = rk4_step(X_prev, ...)
#        X_corr = newton_step(X_pred, ...)
#    - Fallback:
#        If Newton fails to converge (e.g. near-tangent graze or ball jump),
#        the exact algebraic line-circle intersection is used as a robust fallback.
# =============================================================================

using LinearAlgebra

export aim_cue!

# Global tracking state for frame-to-frame continuation
const PREV_TRACKED_BALL = Ref{Int32}(Int32(0))
const PREDICTOR_MODE = Ref{Symbol}(:zero_order) # :rk4 or :zero_order
const PREV_DELTA_X = Ref{Float64}(0.0)
const PREV_DELTA_Y = Ref{Float64}(0.0)

function set_predictor_mode!(mode::Symbol)
    if mode in (:rk4, :zero_order)
        PREDICTOR_MODE[] = mode
    else
        error("Invalid predictor mode: $mode. Expected :rk4 or :zero_order")
    end
end

function get_predictor_mode()::Symbol
    return PREDICTOR_MODE[]
end

# Evaluates the line-circle nonlinear residual system f(x) = [eps_1, eps_2]ᵀ:
# eps_1 = nᵀ (x - p)              (line residual)
# eps_2 = (x - c)ᵀ (x - c) - rad² (circle boundary residual)
@inline @fastmath function f_error(
    x::Tuple{Float64,Float64},
    p::Tuple{Float64,Float64},
    n::Tuple{Float64,Float64},
    c::Tuple{Float64,Float64},
    rad::Float64
)::Tuple{Float64,Float64}
    eps1 = n[1] * (x[1] - p[1]) + n[2] * (x[2] - p[2])
    dx = x[1] - c[1]
    dy = x[2] - c[2]
    eps2 = dx * dx + dy * dy - rad * rad

    return (eps1, eps2)
end

# Evaluates the 2x2 Jacobian matrix J(x) of f(x) with respect to x = (x, y)ᵀ:
#  J = [ n[1]            n[2]          ]
#      [ 2*(x[1]-c[1])   2*(x[2]-c[2]) ]
@inline @fastmath function jacobian(
    x::Tuple{Float64,Float64},
    n::Tuple{Float64,Float64},
    c::Tuple{Float64,Float64}
)::NTuple{4,Float64}
    return (n[1], n[2], 2.0 * (x[1] - c[1]), 2.0 * (x[2] - c[2]))
end

@inline @fastmath function inv_jacobian(J::NTuple{4,Float64})::NTuple{4,Float64}
    j11, j12, j21, j22 = J
    detJ = j11 * j22 - j12 * j21

    # Could error here
    if abs(detJ) < 1e-15
        detJ = detJ >= 0.0 ? 1e-15 : -1e-15
    end

    invdet = 1.0 / detJ

    return (j22 * invdet, -j12 * invdet, -j21 * invdet, j11 * invdet)
end

# Analytically solves the linear system J * dx = b for a 2x2 matrix J
@inline @fastmath function solve_2x2(
    J::NTuple{4,Float64},
    b::Tuple{Float64,Float64}
)::Tuple{Float64,Float64}
    j11, j12, j21, j22 = J
    b1, b2 = b
    detJ = j11 * j22 - j12 * j21

    if abs(detJ) < 1e-15
        detJ = detJ >= 0.0 ? 1e-15 : -1e-15
    end

    invdet = 1.0 / detJ

    return (invdet * (j22 * b1 - j12 * b2), invdet * (-j21 * b1 + j11 * b2))
end

# Computes the partial derivative ∂f/∂t holding x fixed:
# ∂f₁/∂t = (dn/dt)ᵀ (x - p) - nᵀ (dp/dt)
# ∂f₂/∂t = -2 * (x - c)ᵀ (dc/dt)
@inline @fastmath function df_dt(
    x::Tuple{Float64,Float64},
    p::Tuple{Float64,Float64},
    dp_dt::Tuple{Float64,Float64},
    n::Tuple{Float64,Float64},
    dn_dt::Tuple{Float64,Float64},
    c::Tuple{Float64,Float64},
    dc_dt::Tuple{Float64,Float64}
)::Tuple{Float64,Float64}
    d_eps1_dt = dn_dt[1] * (x[1] - p[1]) + dn_dt[2] * (x[2] - p[2]) - (n[1] * dp_dt[1] + n[2] * dp_dt[2])
    d_eps2_dt = -2.0 * ((x[1] - c[1]) * dc_dt[1] + (x[2] - c[2]) * dc_dt[2])

    return (d_eps1_dt, d_eps2_dt)
end

# Evaluates the Davidenko algebraic-differential equation:
# dx/dt = - J(x)⁻¹ * (∂f/∂t)
@inline @fastmath function dx_dt(
    x::Tuple{Float64,Float64},
    p::Tuple{Float64,Float64},
    dp_dt::Tuple{Float64,Float64},
    n::Tuple{Float64,Float64},
    dn_dt::Tuple{Float64,Float64},
    c::Tuple{Float64,Float64},
    dc_dt::Tuple{Float64,Float64}
)::Tuple{Float64,Float64}
    J = jacobian(x, n, c)
    ft = df_dt(x, p, dp_dt, n, dn_dt, c, dc_dt)
    sol = solve_2x2(J, ft)

    return (-sol[1], -sol[2])
end

# Newton-Raphson corrector step:
# X_{k+1} = X_k - J(X_k)⁻¹ * f(X_k)
@inline @fastmath function newton_step(
    x::Tuple{Float64,Float64},
    p::Tuple{Float64,Float64},
    n::Tuple{Float64,Float64},
    c::Tuple{Float64,Float64},
    rad::Float64;
    max_iters::Int=2
)::Tuple{Float64,Float64}
    curr = x

    # Unrolled for 2 iterations
    f = f_error(curr, p, n, c, rad)
    J = jacobian(curr, n, c)
    dx = solve_2x2(J, f)
    curr = (curr[1] - dx[1], curr[2] - dx[2])

    if max_iters > 1
        f = f_error(curr, p, n, c, rad)
        J = jacobian(curr, n, c)
        dx = solve_2x2(J, f)
        curr = (curr[1] - dx[1], curr[2] - dx[2])
    end

    return curr
end

# 1st-order Euler predictor step along Davidenko differential equation:
# x_pred = x + (dx/dt) * dt
@inline @fastmath function euler_step(
    x::Tuple{Float64,Float64},
    p::Tuple{Float64,Float64},
    dp_dt::Tuple{Float64,Float64},
    n::Tuple{Float64,Float64},
    dn_dt::Tuple{Float64,Float64},
    c::Tuple{Float64,Float64},
    dc_dt::Tuple{Float64,Float64},
    dt::Float64
)::Tuple{Float64,Float64}
    k1 = dx_dt(x, p, dp_dt, n, dn_dt, c, dc_dt)

    return (x[1] + k1[1] * dt, x[2] + k1[2] * dt)
end

# 4th-order Runge-Kutta numerical integration step for the Davidenko differential equation:
# dx/dt = - J(x)⁻¹ * (∂f/∂t)
# Integrates from t₀ to t₀ + dt
@inline @fastmath function rk4_step(
    x::Tuple{Float64,Float64},
    p::Tuple{Float64,Float64},
    dp_dt::Tuple{Float64,Float64},
    n::Tuple{Float64,Float64},
    dn_dt::Tuple{Float64,Float64},
    c::Tuple{Float64,Float64},
    dc_dt::Tuple{Float64,Float64},
    dt::Float64
)::Tuple{Float64,Float64}
    dt2 = dt * 0.5
    inv6 = 1.0 / 6.0

    # k1 at t0
    k1 = dx_dt(x, p, dp_dt, n, dn_dt, c, dc_dt)

    # State at t0 + dt/2
    p_mid = (p[1] + dp_dt[1] * dt2, p[2] + dp_dt[2] * dt2)
    n_mid = (n[1] + dn_dt[1] * dt2, n[2] + dn_dt[2] * dt2)
    c_mid = (c[1] + dc_dt[1] * dt2, c[2] + dc_dt[2] * dt2)

    # k2 at t0 + dt/2
    x2 = (x[1] + k1[1] * dt2, x[2] + k1[2] * dt2)
    k2 = dx_dt(x2, p_mid, dp_dt, n_mid, dn_dt, c_mid, dc_dt)

    # k3 at t0 + dt/2
    x3 = (x[1] + k2[1] * dt2, x[2] + k2[2] * dt2)
    k3 = dx_dt(x3, p_mid, dp_dt, n_mid, dn_dt, c_mid, dc_dt)

    # State at t0 + dt
    p_end = (p[1] + dp_dt[1] * dt, p[2] + dp_dt[2] * dt)
    n_end = (n[1] + dn_dt[1] * dt, n[2] + dn_dt[2] * dt)
    c_end = (c[1] + dc_dt[1] * dt, c[2] + dc_dt[2] * dt)

    # k4 at t0 + dt
    x4 = (x[1] + k3[1] * dt, x[2] + k3[2] * dt)
    k4 = dx_dt(x4, p_end, dp_dt, n_end, dn_dt, c_end, dc_dt)

    return (
        x[1] + (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) * dt * inv6,
        x[2] + (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2]) * dt * inv6
    )
end

# 0-order Predictor-Corrector: constant prediction X_pred = x followed by Newton-Raphson correction
@inline @fastmath function track_0th(
    x::Tuple{Float64,Float64},
    p_next::Tuple{Float64,Float64},
    n_next::Tuple{Float64,Float64},
    c_next::Tuple{Float64,Float64},
    rad::Float64
)::Tuple{Float64,Float64}
    return newton_step(x, p_next, n_next, c_next, rad)
end

# 1st-order Predictor-Corrector: 4th-order Runge-Kutta numerical integration predictor
# along the Davidenko differential equation followed by Newton-Raphson correction
@inline @fastmath function track_rk4(
    x::Tuple{Float64,Float64},
    p::Tuple{Float64,Float64},
    dp_dt::Tuple{Float64,Float64},
    n::Tuple{Float64,Float64},
    dn_dt::Tuple{Float64,Float64},
    c::Tuple{Float64,Float64},
    dc_dt::Tuple{Float64,Float64},
    rad::Float64,
    dt::Float64
)::Tuple{Float64,Float64}
    x_pred = rk4_step(x, p, dp_dt, n, dn_dt, c, dc_dt, dt)
    p_next = (p[1] + dp_dt[1] * dt, p[2] + dp_dt[2] * dt)
    n_next = (n[1] + dn_dt[1] * dt, n[2] + dn_dt[2] * dt)
    c_next = (c[1] + dc_dt[1] * dt, c[2] + dc_dt[2] * dt)

    return newton_step(x_pred, p_next, n_next, c_next, rad)
end

# Unified predictor-corrector tracking function using the active `PREDICTOR_MODE[]`
@inline @fastmath function track(
    x::Tuple{Float64,Float64},
    p::Tuple{Float64,Float64},
    dp_dt::Tuple{Float64,Float64},
    n::Tuple{Float64,Float64},
    dn_dt::Tuple{Float64,Float64},
    c::Tuple{Float64,Float64},
    dc_dt::Tuple{Float64,Float64},
    rad::Float64,
    dt::Float64
)::Tuple{Float64,Float64}
    if PREDICTOR_MODE[] == :rk4
        return track_rk4(x, p, dp_dt, n, dn_dt, c, dc_dt, rad, dt)
    else
        p_next = (p[1] + dp_dt[1] * dt, p[2] + dp_dt[2] * dt)
        n_next = (n[1] + dn_dt[1] * dt, n[2] + dn_dt[2] * dt)
        c_next = (c[1] + dc_dt[1] * dt, c[2] + dc_dt[2] * dt)

        return track_0th(x, p_next, n_next, c_next, rad)
    end
end

# Continuation path tracking for a root `ic` from start circle `c_start` to target circle `c_target`.
# In :rk4 mode, integrates Davidenko DE dx/ds = - J⁻¹ (∂f/∂s) using RK4 followed by Newton correction.
# In :zero_order mode, uses constant prediction followed by Newton correction.
@inline @fastmath function track_continuation(
    ic::Tuple{Float64,Float64},
    p::Tuple{Float64,Float64},
    u::Tuple{Float64,Float64},
    n::Tuple{Float64,Float64},
    c_start::Tuple{Float64,Float64},
    c_target::Tuple{Float64,Float64},
    rad::Float64;
    steps::Int=1
)::Tuple{Float64,Float64}
    if PREDICTOR_MODE[] == :zero_order
        return newton_step(ic, p, n, c_target, rad)
    end

    dc_ds = (c_target[1] - c_start[1], c_target[2] - c_start[2])
    x_curr = ic
    ds = 1.0 / Float64(steps)
    s = 0.0

    @inbounds for _ in 1:steps
        ds2 = ds * 0.5
        c_s = (c_start[1] + s * dc_ds[1], c_start[2] + s * dc_ds[2])
        c_mid = (c_start[1] + (s + ds2) * dc_ds[1], c_start[2] + (s + ds2) * dc_ds[2])
        c_next = (c_start[1] + (s + ds) * dc_ds[1], c_start[2] + (s + ds) * dc_ds[2])

        # Davidenko RHS along continuation parameter s:
        # f₁(x, s) = nᵀ (x - p) => ∂f₁/∂s = 0
        # f₂(x, s) = ||x - c(s)||² - rad² => ∂f₂/∂s = -2 (x - c(s))ᵀ (dc/ds)
        J1 = jacobian(x_curr, n, c_s)
        f_s1 = (0.0, -2.0 * ((x_curr[1] - c_s[1]) * dc_ds[1] + (x_curr[2] - c_s[2]) * dc_ds[2]))
        sol1 = solve_2x2(J1, f_s1)
        k1 = (-sol1[1], -sol1[2])

        x2 = (x_curr[1] + k1[1] * ds2, x_curr[2] + k1[2] * ds2)
        J2 = jacobian(x2, n, c_mid)
        f_s2 = (0.0, -2.0 * ((x2[1] - c_mid[1]) * dc_ds[1] + (x2[2] - c_mid[2]) * dc_ds[2]))
        sol2 = solve_2x2(J2, f_s2)
        k2 = (-sol2[1], -sol2[2])

        x3 = (x_curr[1] + k2[1] * ds2, x_curr[2] + k2[2] * ds2)
        J3 = jacobian(x3, n, c_mid)
        f_s3 = (0.0, -2.0 * ((x3[1] - c_mid[1]) * dc_ds[1] + (x3[2] - c_mid[2]) * dc_ds[2]))
        sol3 = solve_2x2(J3, f_s3)
        k3 = (-sol3[1], -sol3[2])

        x4 = (x_curr[1] + k3[1] * ds, x_curr[2] + k3[2] * ds)
        J4 = jacobian(x4, n, c_next)
        f_s4 = (0.0, -2.0 * ((x4[1] - c_next[1]) * dc_ds[1] + (x4[2] - c_next[2]) * dc_ds[2]))
        sol4 = solve_2x2(J4, f_s4)
        k4 = (-sol4[1], -sol4[2])

        x_pred = (
            x_curr[1] + (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) * ds * (1.0 / 6.0),
            x_curr[2] + (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2]) * ds * (1.0 / 6.0)
        )

        x_curr = newton_step(x_pred, p, n, c_next, rad)
        s += ds
    end

    return x_curr
end

@inline @fastmath function track_continuation(
    p::Tuple{Float64,Float64},
    u::Tuple{Float64,Float64},
    n::Tuple{Float64,Float64},
    c::Tuple{Float64,Float64},
    rad::Float64;
    steps::Int=1
)::Tuple{Float64,Float64}
    vx = c[1] - p[1]
    vy = c[2] - p[2]
    t_proj = vx * u[1] + vy * u[2]
    c_start = (p[1] + t_proj * u[1], p[2] + t_proj * u[2])
    r_entry = (p[1] + (t_proj - rad) * u[1], p[2] + (t_proj - rad) * u[2])

    return track_continuation(r_entry, p, u, n, c_start, c, rad; steps=steps)
end

# Computes analytical geometric line-circle intersection roots for the ray p + t*u.
# Returns `(r_entry, r_exit, (t_entry, t_exit))`. Used as initialization and robust fallback
@inline @fastmath function line_circle_roots(
    p::Tuple{Float64,Float64},
    u::Tuple{Float64,Float64},
    c::Tuple{Float64,Float64},
    rad::Float64
)
    vx = c[1] - p[1]
    vy = c[2] - p[2]
    t_proj = vx * u[1] + vy * u[2]
    d_perp_sq = (vx * vx + vy * vy) - t_proj * t_proj
    rad_sq = rad * rad

    if d_perp_sq >= rad_sq
        nan_pt = (NaN, NaN)

        return (nan_pt, nan_pt, (NaN, NaN))
    end

    h = sqrt(max(0.0, rad_sq - d_perp_sq))
    t1 = t_proj - h
    t2 = t_proj + h
    r1 = (p[1] + t1 * u[1], p[2] + t1 * u[2])
    r2 = (p[1] + t2 * u[1], p[2] + t2 * u[2])
    
    return (r1, r2, (t1, t2))
end

# Integration with the code
@inline @fastmath function aim_cue!(data, sid::Int32)
    qx = data.cue_x
    qy = data.cue_y

    # Aiming at arbitrary direction
    if sid == 0
        !data.has_aim && return
        ux = data.aim_dir_x
        uy = data.aim_dir_y

        data.cue_angle = rad2deg(atan(uy, ux)) - 90.0f0

        # When aiming fixed in a direction, the ray extends to the table boundary
        t_bound = Float32(1e6)
        if ux > 1.0f-6
            t_bound = min(t_bound, (data.bound_x - qx) / ux)
        elseif ux < -1.0f-6
            t_bound = min(t_bound, -qx / ux)
        end
        if uy > 1.0f-6
            t_bound = min(t_bound, (data.bound_y - qy) / uy)
        elseif uy < -1.0f-6
            t_bound = min(t_bound, -qy / uy)
        end

        t_min = t_bound
        hit_x = qx + t_bound * ux
        hit_y = qy + t_bound * uy
        # Aiming at ball
    else
        tx = data.ball_x[sid] + data.track_offset_x
        ty = data.ball_y[sid] + data.track_offset_y
        dx = tx - qx
        dy = ty - qy

        data.cue_angle = rad2deg(atan(dy, dx)) - 90.0f0

        # Line-circle intersection for LOS
        dist_sq = dx * dx + dy * dy
        if dist_sq <= 1.0f-6
            data.collision_x = tx
            data.collision_y = ty
            PREV_TRACKED_BALL[] = Int32(0)
            return
        end

        dist = sqrt(dist_sq)
        inv_dist = 1.0f0 / dist
        ux = dx * inv_dist
        uy = dy * inv_dist

        t_min = dist
        hit_x = tx
        hit_y = ty
    end

    rad = Float64(data.ball_radius)
    rad_sq = rad * rad
    dt = Float64(data.dt_target)

    # LOS Ray parameters
    p = (Float64(qx), Float64(qy))
    u = (Float64(ux), Float64(uy))
    n = (Float64(uy), -Float64(ux)) # Unit normal perpendicular to LOS: nᵀ u = 0
    dp_dt = (0.0, 0.0)
    dn_dt = (0.0, 0.0)

    prev_ball_id = PREV_TRACKED_BALL[]
    prev_sol = (Float64(data.collision_x), Float64(data.collision_y))
    best_hit_id = Int32(0)

    # Fast-path prediction using previous collision point:
    # We assume the delta from the previous frame is the same.
    # If Newton moves the solution a lot, we know a collision occurred!
    tracked_fast = false

    if prev_ball_id != 0 && prev_sol[1] != 0.0
        j = prev_ball_id
        bx = Float64(data.ball_x[j])
        by = Float64(data.ball_y[j])
        c_now = (bx, by)

        # Predict position: assume delta from previous frame is the same (or RK4)
        if PREDICTOR_MODE[] == :rk4
            b_vx = Float64(data.ball_vx[j])
            b_vy = Float64(data.ball_vy[j])
            dc_dt = (b_vx, b_vy)
            c_prev = (bx - b_vx * dt, by - b_vy * dt)
            x_pred = rk4_step(prev_sol, p, dp_dt, n, dn_dt, c_prev, dc_dt, dt)
        else
            x_pred = (prev_sol[1] + PREV_DELTA_X[], prev_sol[2] + PREV_DELTA_Y[])
        end

        # Newton corrector step
        x_corr = newton_step(x_pred, p, n, c_now, rad)

        # Check movement: if Newton moved the solution a lot, we know it collided!
        newton_move_sq = (x_corr[1] - x_pred[1])^2 + (x_corr[2] - x_pred[2])^2
        err = f_error(x_corr, p, n, c_now, rad)
        t_hit = (x_corr[1] - p[1]) * u[1] + (x_corr[2] - p[2]) * u[2]

        if newton_move_sq < 0.25 && (err[1]*err[1] + err[2]*err[2]) < 1e-4 && t_hit > 0.0 && t_hit < Float64(t_min)
            # Smooth tracking: no collision occurred on this ball!
            t_min = Float32(t_hit)
            hit_x = Float32(x_corr[1])
            hit_y = Float32(x_corr[2])
            best_hit_id = j
            tracked_fast = true
        end
    end

    # Raycasting hit checking against other balls using Predictor-Corrector DE tracking
    @inbounds @fastmath for j in 1:data.num_balls
        j == sid && continue
        # If tracked_fast succeeded, we already know the hit on best_hit_id
        tracked_fast && j == best_hit_id && continue

        bx = Float64(data.ball_x[j])
        by = Float64(data.ball_y[j])

        vx = bx - p[1]
        vy = by - p[2]

        t_proj = vx * u[1] + vy * u[2]

        # Early rejection: ball is behind cue origin or past current closest collision
        if t_proj <= -rad || t_proj >= Float64(t_min) + rad
            continue
        end

        # Perpendicular distance check
        d_perp_sq = (vx * vx + vy * vy) - t_proj * t_proj
        if d_perp_sq >= rad_sq
            continue
        end

        c_now = (bx, by)
        c_start = (p[1] + t_proj * u[1], p[2] + t_proj * u[2])
        r_entry = (p[1] + (t_proj - rad) * u[1], p[2] + (t_proj - rad) * u[2])
        r_exit  = (p[1] + (t_proj + rad) * u[1], p[2] + (t_proj + rad) * u[2])

        sol1 = track_continuation(r_entry, p, u, n, c_start, c_now, rad)
        sol2 = track_continuation(r_exit,  p, u, n, c_start, c_now, rad)

        found_hit = false
        @inbounds @fastmath for sol in (sol1, sol2)
            err = f_error(sol, p, n, c_now, rad)
            if (err[1] * err[1] + err[2] * err[2]) < 1e-4
                rx = sol[1]
                ry = sol[2]
                t_hit = (rx - p[1]) * u[1] + (ry - p[2]) * u[2]

                if t_hit > 0.0 && t_hit < Float64(t_min)
                    t_min = Float32(t_hit)
                    hit_x = Float32(rx)
                    hit_y = Float32(ry)
                    best_hit_id = j
                    found_hit = true
                end
            end
        end

        # Fallback to exact solution for near-tangent graze where det(J) -> 0
        if !found_hit
            h = sqrt(max(0.0, rad_sq - d_perp_sq))
            t_exact = t_proj - h
            if t_exact > 0.0 && t_exact < Float64(t_min)
                t_min = Float32(t_exact)
                hit_x = Float32(p[1] + t_exact * u[1])
                hit_y = Float32(p[2] + t_exact * u[2])
                best_hit_id = j
            end
        end
    end

    if prev_sol[1] != 0.0
        PREV_DELTA_X[] = Float64(hit_x) - prev_sol[1]
        PREV_DELTA_Y[] = Float64(hit_y) - prev_sol[2]
    end

    data.collision_x = hit_x
    data.collision_y = hit_y
    PREV_TRACKED_BALL[] = best_hit_id
end

end