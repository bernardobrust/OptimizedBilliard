module HomotopySolution

# =============================================================================
# Homotopy Continuation solution for cue aiming in billiards.
# Ported from the Python implementation in billard-homotopy:
#   - linecircle.py (fn, dh1dx, dh2dx, H, Hx, Ht)
#   - path.py (rk4_step, newton_step)
#   - track.py (track)
#   - root_gen_linecircle.py (gen_params, gen_roots)
#
# MATHEMATICAL FORMULATION:
# =============================================================================
# 1. Geometric Objects as Polynomial Systems:
#    - Circle (ball j with center Cⱼ = (bx, by) and radius R):
#        (x - bx)² + (y - by)² - R² = 0
#        => a*(x² + y²) + b*x + c*y + d = 0
#        where a = 1, b = -2*bx, c = -2*by, d = bx² + by² - R²
#    - Line (Line of Sight from cue Q = (qx, qy) with unit direction u = (ux, uy)):
#        uy*(x - qx) - ux*(y - qy) = 0
#        => e*x + f*y + g = 0
#        where e = uy, f = -ux, g = ux*qy - uy*qx
#
# 2. Homotopy Formulation:
#    Given initial system G(x) with parameters params_i and target system F(x)
#    with parameters params_f:
#        H(x, t) = fn(x, (1 - t)*params_i + t*params_f) = 0,   t ∈ [0, 1]
#
# 3. Path Tracking via Davidenko Differential Equation:
#    Differentiating H(x(t), t) = 0 with respect to t gives:
#        Hx(x, t) * (dx/dt) + Ht(x, t) = 0
#        => dx/dt = - Hx(x, t)⁻¹ * Ht(x, t)
#    We use a 4th-order Runge-Kutta (RK4) predictor followed by Newton-Raphson
#    corrections at each step to track the root to t = 1.
# =============================================================================

using LinearAlgebra
using Random

export fn, dh1dx, dh2dx, H, Hx, Ht, rk4_step, newton_step, track, gen_params, gen_roots, aim_cue!

@inline function fn(r, params)
    a, b, c, d, e, f, g = params
    x, y = r[1], r[2]
    f1 = a * (x^2 + y^2) + b * x + c * y + d
    f2 = e * x + f * y + g
    return [f1, f2]
end

@inline function dh1dx(r, params)
    a, b, c, d, e, f, g = params
    x, y = r[1], r[2]
    df1dx = a * (x * 2) + b
    df1dy = a * (y * 2) + c
    return [df1dx, df1dy]
end

@inline function dh2dx(r, params)
    a, b, c, d, e, f, g = params
    df2dx = e
    df2dy = f
    return [df2dx, df2dy]
end

# Homotopy function connecting initial system params_i to target params_f:
# H(x, t) = fn(x, params_i * (1 - t) + params_f * t)
@inline function H(x, t, params_i, params_f)
    pt = params_i .* (1.0 - t) .+ params_f .* t
    return fn(x, pt)
end

# Jacobian matrix of H with respect to x:
#  | dh1/dx  dh1/dy |
#  | dh2/dx  dh2/dy |
@inline function Hx(x, t, params_i, params_f)
    pt = params_i .* (1.0 - t) .+ params_f .* t
    d1 = dh1dx(x, pt)
    d2 = dh2dx(x, pt)
    return [d1[1] d1[2]; d2[1] d2[2]]
end

# Partial derivative of H with respect to continuation parameter t:
# dH/dt = fn(x, -params_i + params_f)
function Ht(x, t, params_i, params_f)
    return fn(x, .- params_i .+ params_f)
end

@inline function solve_2x2(A, b)
    detA = A[1,1] * A[2,2] - A[1,2] * A[2,1]
    if abs(detA) < 1e-15
        return A \ b
    end
    invdet = 1.0 / detA
    return [invdet * (A[2,2] * b[1] - A[1,2] * b[2]),
            invdet * (-A[2,1] * b[1] + A[1,1] * b[2])]
end

# 4th-order Runge-Kutta integration step for homotopic path tracking along
# Davidenko differential equation: dx/dt = - Hx(x, t)^(-1) * Ht(x, t).
@inline function rk4_step(ic, t, p_i, pf, dt=1e-2)
    dx = dt
    J1 = Hx(ic, t, p_i, pf)
    f1 = -solve_2x2(J1, Ht(ic, t, p_i, pf))

    x2 = ic .+ f1 .* (dx / 2)
    t2 = t + dt / 2
    f2 = -solve_2x2(Hx(x2, t2, p_i, pf), Ht(x2, t2, p_i, pf))

    x3 = ic .+ f2 .* (dx / 2)
    t3 = t + dt / 2
    f3 = -solve_2x2(Hx(x3, t3, p_i, pf), Ht(x3, t3, p_i, pf))

    x4 = ic .+ f3 .* dx
    t4 = t + dt
    f4 = -solve_2x2(Hx(x4, t4, p_i, pf), Ht(x4, t4, p_i, pf))

    return ic .+ (f1 .+ 2 .* f2 .+ 2 .* f3 .+ f4) ./ 6
end

# Newton-Raphson corrector step for error reduction:
# x_new = x - Hx(x, t)^(-1) * H(x, t)
function newton_step(ic, t, pi, pf, dumpHx=false)
    J = Hx(ic, t, pi, pf)
    x1 = ic .- solve_2x2(J, H(ic, t, pi, pf))
    it = 0
    while norm(x1 .- ic) > 1e-2 && it < 30
        ic = x1
        J = Hx(ic, t, pi, pf)
        x1 = ic .- solve_2x2(J, H(ic, t, pi, pf))
        it += 1
    end
    det_val = abs(J[1,1] * J[2,2] - J[1,2] * J[2,1])
    return x1, det_val
end

# Path tracker on homotopic continuation from initial root ic to target system pf.
function track(ic, pi, pf)
    thresh = 1.9
    t = 0.0
    dt = 1e-1
    red = 0.5
    inc = 2.0
    p = []
    time_pts = [t]
    absvol = []
    itr = 0
    curr = ic
    @fastmath while t < 1.0 && itr < 100
        step_dt = min(dt, 1.0 - t)
        x1 = rk4_step(curr, t, pi, pf, step_dt)
        next_t = t + step_dt
        y, v = newton_step(x1, next_t, pi, pf, true)

        sub_it = 0
        while norm(y .- x1) > thresh && sub_it < 10
            x1 = y
            y, v = newton_step(x1, next_t, pi, pf, true)
            sub_it += 1
        end

        if norm(y .- curr) > thresh
            dt *= red
        else
            dt = min(dt * inc, 0.5)
            curr = y
            t = next_t
            push!(p, y)
            push!(time_pts, t)
            push!(absvol, v)
        end
        itr += 1
    end
    y_final, v_final = newton_step(curr, 1.0, pi, pf, true)
    return y_final, p, time_pts, absvol
end


# Uniform distribution parameter generation for start system.
@inline function gen_params()
    return rand(Float64, 7) .+ im .* rand(Float64, 7)
end

# Generate roots for the line-circle geometric system analytically.
@inline function gen_roots(params)
    a, b, c, d, e, f, g = params
    if abs(f) >= abs(e)
        A = a * (e^2 + f^2)
        B = 2 * a * e * g + b * f^2 - c * e * f
        C = a * g^2 - c * f * g + d * f^2
        disc = sqrt(Complex(B^2 - 4 * A * C))
        x1 = (-B + disc) / (2 * A)
        y1 = -(e * x1 + g) / f
        x2 = (-B - disc) / (2 * A)
        y2 = -(e * x2 + g) / f
        return [ComplexF64[x1, y1], ComplexF64[x2, y2]]
    else
        A = a * (e^2 + f^2)
        B = 2 * a * f * g + c * e^2 - b * e * f
        C = a * g^2 - b * e * g + d * e^2
        disc = sqrt(Complex(B^2 - 4 * A * C))
        y1 = (-B + disc) / (2 * A)
        x1 = -(f * y1 + g) / e
        y2 = (-B - disc) / (2 * A)
        x2 = -(f * y2 + g) / e
        return [ComplexF64[x1, y1], ComplexF64[x2, y2]]
    end
end

@inline function fn_fast(r::Tuple{ComplexF64, ComplexF64}, p::NTuple{7, ComplexF64})
    x, y = r
    a, b, c, d, e, f, g = p
    f1 = a * (x*x + y*y) + b * x + c * y + d
    f2 = e * x + f * y + g
    return (f1, f2)
end

@inline function dh1dx_fast(r::Tuple{ComplexF64, ComplexF64}, p::NTuple{7, ComplexF64})
    x, y = r
    a, b, c, d, e, f, g = p
    return (2.0 * a * x + b, 2.0 * a * y + c)
end

@inline function dh2dx_fast(p::NTuple{7, ComplexF64})
    return (p[5], p[6])
end

@inline function H_fast(x::Tuple{ComplexF64, ComplexF64}, t::Float64, pi::NTuple{7, ComplexF64}, pf::NTuple{7, ComplexF64})
    one_minus_t = 1.0 - t
    pt = ntuple(i -> pi[i] * one_minus_t + pf[i] * t, 7)
    return fn_fast(x, pt)
end

@inline function Hx_fast(x::Tuple{ComplexF64, ComplexF64}, t::Float64, pi::NTuple{7, ComplexF64}, pf::NTuple{7, ComplexF64})
    one_minus_t = 1.0 - t
    pt = ntuple(i -> pi[i] * one_minus_t + pf[i] * t, 7)
    d1x, d1y = dh1dx_fast(x, pt)
    d2x, d2y = dh2dx_fast(pt)
    return (d1x, d1y, d2x, d2y)
end

@inline function Ht_fast(x::Tuple{ComplexF64, ComplexF64}, pi::NTuple{7, ComplexF64}, pf::NTuple{7, ComplexF64})
    dp = ntuple(i -> pf[i] - pi[i], 7)
    return fn_fast(x, dp)
end

@inline function solve_2x2_fast(J::NTuple{4, ComplexF64}, b::Tuple{ComplexF64, ComplexF64})
    j11, j12, j21, j22 = J
    b1, b2 = b
    detJ = j11 * j22 - j12 * j21
    if abs(detJ) < 1e-15
        detJ = 1e-15 + 0.0im
    end
    invdet = 1.0 / detJ
    return (invdet * (j22 * b1 - j12 * b2), invdet * (-j21 * b1 + j11 * b2))
end

@inline function rk4_step_fast(ic::Tuple{ComplexF64, ComplexF64}, t::Float64, pi::NTuple{7, ComplexF64}, pf::NTuple{7, ComplexF64}, dt::Float64)
    J1 = Hx_fast(ic, t, pi, pf)
    Ht1 = Ht_fast(ic, pi, pf)
    sol1 = solve_2x2_fast(J1, Ht1)
    k1 = (-sol1[1], -sol1[2])

    dt2 = dt * 0.5
    t2 = t + dt2
    x2 = (ic[1] + k1[1] * dt2, ic[2] + k1[2] * dt2)
    J2 = Hx_fast(x2, t2, pi, pf)
    Ht2 = Ht_fast(x2, pi, pf)
    sol2 = solve_2x2_fast(J2, Ht2)
    k2 = (-sol2[1], -sol2[2])

    x3 = (ic[1] + k2[1] * dt2, ic[2] + k2[2] * dt2)
    J3 = Hx_fast(x3, t2, pi, pf)
    Ht3 = Ht_fast(x3, pi, pf)
    sol3 = solve_2x2_fast(J3, Ht3)
    k3 = (-sol3[1], -sol3[2])

    t4 = t + dt
    x4 = (ic[1] + k3[1] * dt, ic[2] + k3[2] * dt)
    J4 = Hx_fast(x4, t4, pi, pf)
    Ht4 = Ht_fast(x4, pi, pf)
    sol4 = solve_2x2_fast(J4, Ht4)
    k4 = (-sol4[1], -sol4[2])

    inv6 = 1.0 / 6.0
    return (ic[1] + (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) * dt * inv6,
            ic[2] + (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2]) * dt * inv6)
end

@inline function newton_step_fast(ic::Tuple{ComplexF64, ComplexF64}, t::Float64, pi::NTuple{7, ComplexF64}, pf::NTuple{7, ComplexF64})
    x = ic

    @inbounds @fastmath for _ in 1:20
        J = Hx_fast(x, t, pi, pf)
        h = H_fast(x, t, pi, pf)
        dx = solve_2x2_fast(J, h)
        x_next = (x[1] - dx[1], x[2] - dx[2])
        diff = abs(dx[1])^2 + abs(dx[2])^2
        x = x_next
        if diff < 1e-4
            break
        end
    end
    return x
end

@inline function track_fast(ic::Tuple{ComplexF64, ComplexF64}, pi::NTuple{7, ComplexF64}, pf::NTuple{7, ComplexF64})
    thresh_sq = 1.9^2
    t = 0.0
    dt = 0.1
    curr = ic
    itr = 0

    @fastmath while t < 1.0 && itr < 100
        step_dt = min(dt, 1.0 - t)
        pred = rk4_step_fast(curr, t, pi, pf, step_dt)
        next_t = t + step_dt
        corr = newton_step_fast(pred, next_t, pi, pf)

        diff_pred = abs(corr[1] - pred[1])^2 + abs(corr[2] - pred[2])^2
        sub_it = 0
        while diff_pred > thresh_sq && sub_it < 10
            pred = corr
            corr = newton_step_fast(pred, next_t, pi, pf)
            diff_pred = abs(corr[1] - pred[1])^2 + abs(corr[2] - pred[2])^2
            sub_it += 1
        end

        diff_curr = abs(corr[1] - curr[1])^2 + abs(corr[2] - curr[2])^2
        if diff_curr > thresh_sq
            dt *= 0.5
        else
            dt = min(dt * 2.0, 0.5)
            curr = corr
            t = next_t
        end
        itr += 1
    end

    return newton_step_fast(curr, 1.0, pi, pf)
end

# Integration with the system
@inline function aim_cue!(data, sid::Int32)
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

        # V = P_track(t) - Q, θ = atan2(V_y, V_x), cue_angle = θ - 90°
        data.cue_angle = rad2deg(atan(dy, dx)) - 90.0f0

        # Line-circle intersection for LOS
        dist_sq = dx * dx + dy * dy
        if dist_sq <= 1.0f-6
            data.collision_x = tx
            data.collision_y = ty
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

    rad = data.ball_radius
    rad_sq = rad * rad

    # Homotopy continuation hit checking against other balls
    @inbounds @fastmath for j in 1:data.num_balls
        j == sid && continue

        bx = data.ball_x[j]
        by = data.ball_y[j]

        vx = bx - qx
        vy = by - qy

        t_proj = vx * ux + vy * uy

        # Early rejection: ball is behind cue origin or past current closest collision
        if t_proj <= -rad || t_proj >= t_min + rad
            continue
        end

        # Perpendicular distance check
        d_perp_sq = (vx * vx + vy * vy) - t_proj * t_proj
        if d_perp_sq >= rad_sq
            continue
        end

        # Target polynomial system for ball j and cue ray:
        #   Circle: x^2 + y^2 - 2bx*x - 2by*y + (bx^2 + by^2 - rad^2) = 0
        #   Line:   uy*x - ux*y + (ux*qy - uy*qx) = 0
        pf = (
            1.0 + 0.0im,
            -2.0 * Float64(bx) + 0.0im,
            -2.0 * Float64(by) + 0.0im,
            Float64(bx * bx + by * by - rad * rad) + 0.0im,
            Float64(uy) + 0.0im,
            -Float64(ux) + 0.0im,
            Float64(ux * qy - uy * qx) + 0.0im
        )

        # Start system: circle centered along the ray at distance t_proj
        scx = Float64(qx + t_proj * ux)
        scy = Float64(qy + t_proj * uy)
        pi = (
            1.0 + 0.0im,
            -2.0 * scx + 0.0im,
            -2.0 * scy + 0.0im,
            (scx * scx + scy * scy - Float64(rad * rad)) + 0.0im,
            Float64(uy) + 0.0im,
            -Float64(ux) + 0.0im,
            Float64(ux * qy - uy * qx) + 0.0im
        )

        # Start roots along the ray
        r_entry = (ComplexF64(qx + (t_proj - rad) * ux), ComplexF64(qy + (t_proj - rad) * uy))
        r_exit  = (ComplexF64(qx + (t_proj + rad) * ux), ComplexF64(qy + (t_proj + rad) * uy))

        sol1 = track_fast(r_entry, pi, pf)
        sol2 = track_fast(r_exit,  pi, pf)

        @fastmath for sol in (sol1, sol2)
            if abs(imag(sol[1])) < 1.0 && abs(imag(sol[2])) < 1.0
                rx = Float32(real(sol[1]))
                ry = Float32(real(sol[2]))
                t_hit = (rx - qx) * ux + (ry - qy) * uy
                if t_hit > 0.0f0 && t_hit < t_min
                    t_min = t_hit
                    hit_x = rx
                    hit_y = ry
                end
            end
        end
    end

    data.collision_x = hit_x
    data.collision_y = hit_y
end

end
