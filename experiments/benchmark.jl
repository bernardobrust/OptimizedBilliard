# =============================================================================
# OptimizedBilliard Benchmark & Numerical Validation Suite
# Measures simulated updates using the game's ball integration, wall reflection,
# pair collisions, and aim solver (rendering and event handling excluded), and
# runs numerical experiments on:
#   1. Ball count scaling (N = 4, 8, 16, 32, 64) over 100,000 steps
#   2. Operational scenarios (Clear, Blocked, Dense, Moving LOS) over 100,000 steps
#   3. Standard 16-ball latency summary over 100,000 steps
#   4. Central loop fast-path hit rate & branch breakdown
#   5. Very small timestep (low `dt`) accuracy, fast-path, and latency scaling
#   6. Constant predictor Newton iteration ablation (2 vs 1 vs 0 iterations)
#      and `track_continuation` mathematical/numerical validation
# =============================================================================

include("../src/Data.jl")
include("../src/SimpleSolution.jl")
include("../src/HomotopySolution.jl")
include("../src/DESolution.jl")
include("../src/DEConstantSolution.jl")
include("../src/DEEulerSolution.jl")
include("../src/DERK4Solution.jl")
include("plot_utils.jl")

using .Data
using .SimpleSolution
using .HomotopySolution
using .DESolution
using .DEConstantSolution, .DEEulerSolution, .DERK4Solution
using .PlotUtils
using Printf
using Random

# Number of simulated physics + aiming updates per benchmark run (increased from 15,000 to 100,000)
const REPETITIONS = 100_000

# Creates a deterministic `DataPlex` scene with `num_balls` balls for reproducible benchmarking
function create_data(num_balls::Int32; blocked::Bool=true, dt_target::Float64=1.0/120.0)
    ball_radius = 18.0f0
    ball_x = zeros(Float32, num_balls)
    ball_y = zeros(Float32, num_balls)
    ball_vx = zeros(Float32, num_balls)
    ball_vy = zeros(Float32, num_balls)

    # Ball 1 is the tracked target ball
    ball_x[1] = 700.0f0
    ball_y[1] = 500.0f0
    ball_vx[1] = -40.0f0
    ball_vy[1] = 30.0f0

    if blocked && num_balls >= 2
        # Ball 2 directly blocks the LOS between cue (100, 200) and target (700, 500)
        ball_x[2] = 400.0f0
        ball_y[2] = 350.0f0
        ball_vx[2] = 60.0f0
        ball_vy[2] = -45.0f0
    elseif num_balls >= 2
        # Ball 2 placed away from the initial sightline
        ball_x[2] = 100.0f0
        ball_y[2] = 700.0f0
    end

    # Deterministically seed remaining balls
    rng = MersenneTwister(42)
    for i in 3:num_balls
        ball_x[i] = 100.0f0 + rand(rng, Float32) * 1000.0f0
        ball_y[i] = 100.0f0 + rand(rng, Float32) * 600.0f0
        ball_vx[i] = -80.0f0 + rand(rng, Float32) * 160.0f0
        ball_vy[i] = -80.0f0 + rand(rng, Float32) * 160.0f0
    end

    Data.DataPlex(
        UInt64(0), UInt64(1000), dt_target,
        1200.0f0, 800.0f0,
        num_balls, ball_radius,
        ball_x, ball_y, ball_vx, ball_vy,
        100.0f0, 200.0f0, 16.0f0, 220.0f0, 0.0f0,
        false, 0.0f0, 0.0f0, Int32(1), 0.0f0, 0.0f0,
        0.0f0, 0.0f0, 0.0f0, 0.0f0, 0.0f0, 0.0f0, true,
        Ptr{Data.SDL_Window}(C_NULL), Ptr{Data.SDL_Renderer}(C_NULL), Ptr{Data.SDL_Texture}(C_NULL),
        false, Int32(0), :simple, Ptr{Data.TTF_Font}(C_NULL),
        0.0, 0.0, zeros(Float64, 100), 1, 1, 100
    )
end

# Resets all global continuation tracking state in `DESolution` before a fresh run
function reset_de_state!()
    DESolution.PREV_TRACKED_BALL[] = 0
    DESolution.PREV_DELTA_X[] = 0.0
    DESolution.PREV_DELTA_Y[] = 0.0
end

# Advances ball linear kinematics, specular wall reflections, and pairwise elastic collisions
function advance_game_physics!(data)
    dt = Float32(data.dt_target)
    rad = data.ball_radius
    max_x, max_y = data.bound_x - rad, data.bound_y - rad
    @inbounds for i in 1:data.num_balls
        x = data.ball_x[i] + data.ball_vx[i] * dt
        y = data.ball_y[i] + data.ball_vy[i] * dt
        vx, vy = data.ball_vx[i], data.ball_vy[i]
        if x < rad
            x = rad; vx = -vx
        elseif x > max_x
            x = max_x; vx = -vx
        end
        if y < rad
            y = rad; vy = -vy
        elseif y > max_y
            y = max_y; vy = -vy
        end
        data.ball_x[i], data.ball_y[i] = x, y
        data.ball_vx[i], data.ball_vy[i] = vx, vy
    end

    min_dist = 2.0f0 * rad
    min_dist_sq = min_dist * min_dist
    @inbounds for i in 1:(data.num_balls-1)
        x1, y1 = data.ball_x[i], data.ball_y[i]
        vx1, vy1 = data.ball_vx[i], data.ball_vy[i]
        for j in (i+1):data.num_balls
            x2, y2 = data.ball_x[j], data.ball_y[j]
            dx, dy = x2 - x1, y2 - y1
            dist_sq = dx * dx + dy * dy
            if dist_sq < min_dist_sq
                dist = sqrt(dist_sq)
                nx, ny = dist == 0.0f0 ? (1.0f0, 0.0f0) : (dx / dist, dy / dist)
                overlap = 0.5f0 * (min_dist - dist)
                x1 -= nx * overlap; y1 -= ny * overlap
                data.ball_x[j] = x2 + nx * overlap
                data.ball_y[j] = y2 + ny * overlap
                vx2, vy2 = data.ball_vx[j], data.ball_vy[j]
                rel_speed = (vx1 - vx2) * nx + (vy1 - vy2) * ny
                if rel_speed > 0.0f0
                    vx1 -= rel_speed * nx; vy1 -= rel_speed * ny
                    data.ball_vx[j] = vx2 + rel_speed * nx
                    data.ball_vy[j] = vy2 + rel_speed * ny
                end
            end
        end
        data.ball_x[i], data.ball_y[i] = x1, y1
        data.ball_vx[i], data.ball_vy[i] = vx1, vy1
    end
end

# Measures average nanoseconds per simulated update (`advance_game_physics!` + `f!`) over `reps` steps
function benchmark_func(f!, source_data, sid::Int32; reps::Int=REPETITIONS)
    data = deepcopy(source_data)
    DESolution.set_algorithmic_dt!(data.dt_target)
    initial_x, initial_y = copy(data.ball_x), copy(data.ball_y)
    initial_vx, initial_vy = copy(data.ball_vx), copy(data.ball_vy)
    # Match the game's fixed-rate aiming update: advance physics, then aim once.
    step!() = (advance_game_physics!(data); f!(data, sid))
    reset!() = begin
        copyto!(data.ball_x, initial_x); copyto!(data.ball_y, initial_y)
        copyto!(data.ball_vx, initial_vx); copyto!(data.ball_vy, initial_vy)
        data.collision_x = 0.0f0; data.collision_y = 0.0f0
        reset_de_state!()
    end
    # Warm-up pass for JIT compilation
    reset!()
    for _ in 1:200
        step!()
    end
    # Timed measurement pass
    reset!()
    t0 = time_ns()
    for _ in 1:reps
        step!()
    end
    t_total = time_ns() - t0
    return t_total / reps
end


# =============================================================================
# Experiment 4 Helper: Instruments `DESolution.aim_cue!` to count exact branch
# frequencies inside the temporal fast-path and central candidate loop.
# =============================================================================
function analyze_fastpath_and_loop(source_data, sid::Int32, mode::Symbol; reps::Int=REPETITIONS)
    data = deepcopy(source_data)
    reset_de_state!()
    DESolution.set_algorithmic_dt!(data.dt_target)
    DESolution.set_predictor_mode!(mode)

    rad = Float64(data.ball_radius)
    rad_sq = rad * rad
    dt = data.dt_target

    fast_eligible = 0       # Frames where `prev_ball_id != 0` (blocker was active in previous frame)
    fast_taken = 0          # Frames where `tracked_fast == true` (fast-path accepted!)
    fail_move = 0           # Fast-path rejections due to `newton_move_sq >= 0.25`
    fail_err = 0            # Fast-path rejections due to `||f(x_corr)||² >= 1e-4`
    fail_t = 0              # Fast-path rejections due to `t_hit` out of range

    # Central loop (`for j in 1:num_balls`) branch counters
    loop_total_iters = 0
    loop_skip_sid = 0       # `j == sid && continue`
    loop_skip_fastpath = 0  # `tracked_fast && j == best_hit_id && continue`
    loop_reject_tproj = 0   # `t_proj <= -rad || t_proj >= t_min + rad`
    loop_reject_dperp = 0   # `d_perp_sq >= rad_sq`
    loop_cont_calls = 0     # Candidate balls reaching `track_continuation`
    loop_cont_conv = 0      # `track_continuation` converged (`||f(sol1)||² < 1e-4`)
    loop_fallback = 0       # Fallback to exact `sqrt(rad_sq - d_perp_sq)` (`!found_hit`)

    for _ in 1:reps
        advance_game_physics!(data)

        qx = data.cue_x; qy = data.cue_y
        tx = data.ball_x[sid] + data.track_offset_x
        ty = data.ball_y[sid] + data.track_offset_y
        dx = tx - qx; dy = ty - qy
        dist = sqrt(dx * dx + dy * dy)
        inv_dist = 1.0f0 / dist
        ux = dx * inv_dist; uy = dy * inv_dist
        t_min = dist

        p = (Float64(qx), Float64(qy))
        u = (Float64(ux), Float64(uy))
        n = (Float64(uy), -Float64(ux))
        dp_dt = (0.0, 0.0)
        tvx = Float64(data.ball_vx[sid]); tvy = Float64(data.ball_vy[sid])
        inv_d = Float64(inv_dist)
        v_dot_u = tvx * u[1] + tvy * u[2]
        dn_dt = ((tvy - v_dot_u * u[2]) * inv_d, -(tvx - v_dot_u * u[1]) * inv_d)

        prev_ball_id = DESolution.PREV_TRACKED_BALL[]
        prev_sol = (Float64(data.collision_x), Float64(data.collision_y))
        best_hit_id = Int32(0)
        tracked_fast = false

        if prev_ball_id != 0 && prev_sol[1] != 0.0
            fast_eligible += 1
            j = prev_ball_id
            bx = Float64(data.ball_x[j]); by = Float64(data.ball_y[j])
            c_now = (bx, by)
            if mode == :rk4 || mode == :euler
                b_vx = Float64(data.ball_vx[j]); b_vy = Float64(data.ball_vy[j])
                dc_dt = (b_vx, b_vy)
                c_prev = (bx - b_vx * dt, by - b_vy * dt)
                n_prev = (n[1] - dn_dt[1] * dt, n[2] - dn_dt[2] * dt)
                x_pred = mode == :rk4 ?
                    DESolution.rk4_step(prev_sol, p, dp_dt, n_prev, dn_dt, c_prev, dc_dt, dt) :
                    DESolution.euler_step(prev_sol, p, dp_dt, n_prev, dn_dt, c_prev, dc_dt, dt)
                x_corr = DESolution.newton_step(x_pred, p, n, c_now, rad)
            else
                x_pred = (prev_sol[1] + DESolution.PREV_DELTA_X[], prev_sol[2] + DESolution.PREV_DELTA_Y[])
                x_corr = DESolution.newton_step(x_pred, p, n, c_now, rad)
            end
            newton_move_sq = (x_corr[1] - x_pred[1])^2 + (x_corr[2] - x_pred[2])^2
            err = DESolution.f_error(x_corr, p, n, c_now, rad)
            err_sq = err[1]^2 + err[2]^2
            t_hit = (x_corr[1] - p[1]) * u[1] + (x_corr[2] - p[2]) * u[2]

            if newton_move_sq < 0.25 && err_sq < 1e-4 && t_hit > 0.0 && t_hit < Float64(t_min)
                t_min = Float32(t_hit)
                best_hit_id = j
                tracked_fast = true
                fast_taken += 1
            else
                newton_move_sq >= 0.25 && (fail_move += 1)
                err_sq >= 1e-4 && (fail_err += 1)
                !(t_hit > 0.0 && t_hit < Float64(t_min)) && (fail_t += 1)
            end
        end

        # Instrument the central candidate loop
        for j in 1:data.num_balls
            loop_total_iters += 1
            if j == sid
                loop_skip_sid += 1
                continue
            end
            if tracked_fast && j == best_hit_id
                loop_skip_fastpath += 1
                continue
            end
            bx = Float64(data.ball_x[j]); by = Float64(data.ball_y[j])
            vx = bx - p[1]; vy = by - p[2]
            t_proj = vx * u[1] + vy * u[2]
            if t_proj <= -rad || t_proj >= Float64(t_min) + rad
                loop_reject_tproj += 1
                continue
            end
            d_perp_sq = (vx * vx + vy * vy) - t_proj * t_proj
            if d_perp_sq >= rad_sq
                loop_reject_dperp += 1
                continue
            end
            loop_cont_calls += 1
            c_now = (bx, by)
            c_start = (p[1] + t_proj * u[1], p[2] + t_proj * u[2])
            r_entry = (p[1] + (t_proj - rad) * u[1], p[2] + (t_proj - rad) * u[2])
            sol1 = DESolution.track_continuation(r_entry, p, u, n, c_start, c_now, rad)
            err = DESolution.f_error(sol1, p, n, c_now, rad)
            found_hit = false
            if (err[1]^2 + err[2]^2) < 1e-4
                loop_cont_conv += 1
                found_hit = true
                t_hit = (sol1[1] - p[1]) * u[1] + (sol1[2] - p[2]) * u[2]
                if t_hit > 0.0 && t_hit < Float64(t_min)
                    t_min = Float32(t_hit)
                    best_hit_id = Int32(j)
                end
            end
            if !found_hit
                loop_fallback += 1
                h = sqrt(max(0.0, rad_sq - d_perp_sq))
                t_exact = t_proj - h
                if t_exact > 0.0 && t_exact < Float64(t_min)
                    t_min = Float32(t_exact)
                    best_hit_id = Int32(j)
                end
            end

        end

        # Advance state via actual solver call so `PREV_TRACKED_BALL` and `PREV_DELTA` stay synchronized
        DESolution.aim_cue!(data, sid)
    end

    return (
        reps = reps,
        fast_eligible = fast_eligible,
        fast_taken = fast_taken,
        fast_hit_rate_eligible = 100.0 * fast_taken / max(1, fast_eligible),
        fast_hit_rate_all = 100.0 * fast_taken / reps,
        fail_move = fail_move,
        fail_err = fail_err,
        fail_t = fail_t,
        loop_total_iters = loop_total_iters,
        loop_skip_sid = loop_skip_sid,
        loop_skip_fastpath = loop_skip_fastpath,
        loop_reject_tproj = loop_reject_tproj,
        loop_reject_dperp = loop_reject_dperp,
        loop_cont_calls = loop_cont_calls,
        loop_cont_conv = loop_cont_conv,
        loop_fallback = loop_fallback
    )
end

# =============================================================================
# Experiment 5 & 6 Helper: Evaluates trajectory accuracy (RMSE and Max Error vs
# exact Float64 reference and Simple Float32 baseline) and fast-path hit rate
# over a 5.0-second deterministic scene for any timestep `dt_val` and solver.
# =============================================================================
function make_trajectory_scene(dt_val::Float64)
    n = Int32(16)
    ball_x = zeros(Float32, n); ball_y = zeros(Float32, n)
    ball_vx = zeros(Float32, n); ball_vy = zeros(Float32, n)
    ball_x[1:3] .= Float32[760, 415, 800]
    ball_y[1:3] .= Float32[520, 365, 200]
    ball_vx[1:3] .= Float32[-35, 12, -20]
    ball_vy[1:3] .= Float32[22, -8, 15]
    for i in 4:n
        ball_x[i] = 50f0 + 80f0 * (i - 4)
        ball_y[i] = 750f0
    end
    Data.DataPlex(UInt64(0), UInt64(1), dt_val, 1200f0, 800f0, n, 18f0,
        ball_x, ball_y, ball_vx, ball_vy,
        100f0, 200f0, 16f0, 220f0, 0f0, false, 0f0, 0f0, Int32(1),
        12f0, -4f0, 0f0, 0f0, 0f0, 0f0, 0f0, 0f0, true,
        Ptr{Data.SDL_Window}(C_NULL), Ptr{Data.SDL_Renderer}(C_NULL),
        Ptr{Data.SDL_Texture}(C_NULL), false, Int32(0), :simple,
        Ptr{Data.TTF_Font}(C_NULL), 0.0, 0.0, zeros(100), 1, 1, 100)
end

function advance_trajectory_scene!(d, dt_val::Float64)
    dt = Float32(dt_val)
    for i in 1:d.num_balls
        x = d.ball_x[i] + d.ball_vx[i] * dt
        y = d.ball_y[i] + d.ball_vy[i] * dt
        if x < d.ball_radius
            d.ball_x[i] = d.ball_radius; d.ball_vx[i] = -d.ball_vx[i]
        elseif x > d.bound_x - d.ball_radius
            d.ball_x[i] = d.bound_x - d.ball_radius; d.ball_vx[i] = -d.ball_vx[i]
        else
            d.ball_x[i] = x
        end
        if y < d.ball_radius
            d.ball_y[i] = d.ball_radius; d.ball_vy[i] = -d.ball_vy[i]
        elseif y > d.bound_y - d.ball_radius
            d.ball_y[i] = d.bound_y - d.ball_radius; d.ball_vy[i] = -d.ball_vy[i]
        else
            d.ball_y[i] = y
        end
    end
end

# Helper for Experiment 6 ablation: runs Constant (:zero_order) solver with 0, 1, or 2 Newton steps
# by calling the single-iteration `DESolution.newton_step` 0, 1, or 2 times.
@inline @fastmath function aim_cue_const_ablation!(data, sid::Int32, ::Val{NIters}) where {NIters}
    NIters == 1 && return DEConstantSolution.aim_cue!(data, sid)
    qx = data.cue_x; qy = data.cue_y
    tx = data.ball_x[sid] + data.track_offset_x
    ty = data.ball_y[sid] + data.track_offset_y
    dx = tx - qx; dy = ty - qy
    data.cue_angle = rad2deg(atan(dy, dx)) - 90.0f0
    dist_sq = dx * dx + dy * dy
    if dist_sq <= 1.0f-6
        data.collision_x = tx; data.collision_y = ty
        DESolution.PREV_TRACKED_BALL[] = Int32(0)
        return
    end
    dist = sqrt(dist_sq); inv_dist = 1.0f0 / dist
    ux = dx * inv_dist; uy = dy * inv_dist
    t_min = dist; hit_x = tx; hit_y = ty
    rad = Float64(data.ball_radius); rad_sq = rad * rad
    p = (Float64(qx), Float64(qy)); u = (Float64(ux), Float64(uy)); n = (Float64(uy), -Float64(ux))
    prev_ball_id = DESolution.PREV_TRACKED_BALL[]
    prev_sol = (Float64(data.collision_x), Float64(data.collision_y))
    best_hit_id = Int32(0); tracked_fast = false

    if prev_ball_id != 0 && prev_sol[1] != 0.0
        j = prev_ball_id
        c_now = (Float64(data.ball_x[j]), Float64(data.ball_y[j]))
        x_pred = (prev_sol[1] + DESolution.PREV_DELTA_X[], prev_sol[2] + DESolution.PREV_DELTA_Y[])
        x_corr = x_pred
        if NIters >= 1
            x_corr = DESolution.newton_step(x_corr, p, n, c_now, rad)
        end
        if NIters >= 2
            x_corr = DESolution.newton_step(x_corr, p, n, c_now, rad)
        end
        newton_move_sq = (x_corr[1] - x_pred[1])^2 + (x_corr[2] - x_pred[2])^2
        err = DESolution.f_error(x_corr, p, n, c_now, rad)
        t_hit = (x_corr[1] - p[1]) * u[1] + (x_corr[2] - p[2]) * u[2]
        if newton_move_sq < 0.25 && (err[1]^2 + err[2]^2) < 1e-4 && t_hit > 0.0 && t_hit < Float64(t_min)
            t_min = Float32(t_hit); hit_x = Float32(x_corr[1]); hit_y = Float32(x_corr[2])
            best_hit_id = j; tracked_fast = true
        end
    end

    @inbounds for j in 1:data.num_balls
        j == sid && continue
        tracked_fast && j == best_hit_id && continue
        bx = Float64(data.ball_x[j]); by = Float64(data.ball_y[j])
        vx = bx - p[1]; vy = by - p[2]
        t_proj = vx * u[1] + vy * u[2]
        (t_proj <= -rad || t_proj >= Float64(t_min) + rad) && continue
        d_perp_sq = (vx * vx + vy * vy) - t_proj * t_proj
        d_perp_sq >= rad_sq && continue
        c_now = (bx, by)
        sol1 = (p[1] + (t_proj - rad) * u[1], p[2] + (t_proj - rad) * u[2])
        if NIters >= 1
            sol1 = DESolution.newton_step(sol1, p, n, c_now, rad)
        end
        if NIters >= 2
            sol1 = DESolution.newton_step(sol1, p, n, c_now, rad)
        end
        err = DESolution.f_error(sol1, p, n, c_now, rad)
        found_hit = false
        if (err[1]^2 + err[2]^2) < 1e-4
            found_hit = true
            t_hit = (sol1[1] - p[1]) * u[1] + (sol1[2] - p[2]) * u[2]
            if t_hit > 0.0 && t_hit < Float64(t_min)
                t_min = Float32(t_hit); hit_x = Float32(sol1[1]); hit_y = Float32(sol1[2]); best_hit_id = Int32(j)
            end
        end
        if !found_hit
            h = sqrt(max(0.0, rad_sq - d_perp_sq))
            t_exact = t_proj - h
            if t_exact > 0.0 && t_exact < Float64(t_min)
                t_min = Float32(t_exact)
                hit_x = Float32(p[1] + t_exact * u[1]); hit_y = Float32(p[2] + t_exact * u[2]); best_hit_id = Int32(j)
            end
        end
    end

    if prev_sol[1] != 0.0 && best_hit_id != 0 && best_hit_id == prev_ball_id
        DESolution.PREV_DELTA_X[] = Float64(hit_x) - prev_sol[1]
        DESolution.PREV_DELTA_Y[] = Float64(hit_y) - prev_sol[2]
    else
        DESolution.PREV_DELTA_X[] = 0.0; DESolution.PREV_DELTA_Y[] = 0.0
    end
    data.collision_x = hit_x; data.collision_y = hit_y
    DESolution.PREV_TRACKED_BALL[] = best_hit_id
end

function evaluate_trajectory_accuracy(aim!, dt_val::Float64; mode=nothing, newton_iters::Int=1, total_time::Float64=2.0)
    steps = round(Int, total_time / dt_val)
    d_sim = make_trajectory_scene(dt_val)
    d_ref = make_trajectory_scene(dt_val)
    reset_de_state!()
    DESolution.set_algorithmic_dt!(dt_val)
    mode === nothing || DESolution.set_predictor_mode!(mode)

    max_err_simple = 0.0
    sum_sq_simple = 0.0
    max_err_f64 = 0.0
    sum_sq_f64 = 0.0
    fast_eligible = 0
    fast_taken = 0

    rad = Float64(d_sim.ball_radius)

    for _ in 1:steps
        # Check fast-path eligibility before calling aim!
        prev_ball_id = DESolution.PREV_TRACKED_BALL[]
        prev_sol = (Float64(d_sim.collision_x), Float64(d_sim.collision_y))
        if mode !== nothing && prev_ball_id != 0 && prev_sol[1] != 0.0
            fast_eligible += 1
            j = prev_ball_id
            qx = d_sim.cue_x; qy = d_sim.cue_y
            tx = d_sim.ball_x[1] + d_sim.track_offset_x
            ty = d_sim.ball_y[1] + d_sim.track_offset_y
            dx = tx - qx; dy = ty - qy
            dist = sqrt(dx * dx + dy * dy); inv_dist = 1.0f0 / dist
            ux = dx * inv_dist; uy = dy * inv_dist
            p = (Float64(qx), Float64(qy)); u = (Float64(ux), Float64(uy)); n = (Float64(uy), -Float64(ux))
            tvx = Float64(d_sim.ball_vx[1]); tvy = Float64(d_sim.ball_vy[1]); inv_d = Float64(inv_dist)
            v_dot_u = tvx * u[1] + tvy * u[2]
            dn_dt = ((tvy - v_dot_u * u[2]) * inv_d, -(tvx - v_dot_u * u[1]) * inv_d)
            bx = Float64(d_sim.ball_x[j]); by = Float64(d_sim.ball_y[j]); c_now = (bx, by)
            if mode == :rk4 || mode == :euler
                b_vx = Float64(d_sim.ball_vx[j]); b_vy = Float64(d_sim.ball_vy[j])
                c_prev = (bx - b_vx * dt_val, by - b_vy * dt_val)
                n_prev = (n[1] - dn_dt[1] * dt_val, n[2] - dn_dt[2] * dt_val)
                x_pred = mode == :rk4 ?
                    DESolution.rk4_step(prev_sol, p, (0.0, 0.0), n_prev, dn_dt, c_prev, (b_vx, b_vy), dt_val) :
                    DESolution.euler_step(prev_sol, p, (0.0, 0.0), n_prev, dn_dt, c_prev, (b_vx, b_vy), dt_val)
                x_corr = DESolution.newton_step(x_pred, p, n, c_now, rad)
            else
                x_pred = (prev_sol[1] + DESolution.PREV_DELTA_X[], prev_sol[2] + DESolution.PREV_DELTA_Y[])
                x_corr = x_pred
                for _ in 1:newton_iters
                    x_corr = DESolution.newton_step(x_corr, p, n, c_now, rad)
                end
            end
            move_sq = (x_corr[1] - x_pred[1])^2 + (x_corr[2] - x_pred[2])^2
            err = DESolution.f_error(x_corr, p, n, c_now, rad)
            t_hit = (x_corr[1] - p[1]) * u[1] + (x_corr[2] - p[2]) * u[2]
            if move_sq < 0.25 && (err[1]^2 + err[2]^2) < 1e-4 && t_hit > 0.0 && t_hit < Float64(dist)
                fast_taken += 1
            end
        end

        if mode == :zero_order && newton_iters != 1
            aim_cue_const_ablation!(d_sim, Int32(1), Val(newton_iters))
        else
            aim!(d_sim, Int32(1))
        end
        SimpleSolution.aim_cue!(d_ref, Int32(1))

        # Exact Float64 reference (quantized to Float32 output storage to isolate algorithmic error)
        qx64 = Float64(d_ref.cue_x); qy64 = Float64(d_ref.cue_y)
        tx64 = Float64(d_ref.ball_x[1] + d_ref.track_offset_x)
        ty64 = Float64(d_ref.ball_y[1] + d_ref.track_offset_y)
        dx64 = tx64 - qx64; dy64 = ty64 - qy64; dist64 = hypot(dx64, dy64)
        ux64 = dx64 / dist64; uy64 = dy64 / dist64
        t_min64 = dist64; hx64 = tx64; hy64 = ty64
        for j in 2:d_ref.num_balls
            r1, _, (t1, _) = DESolution.line_circle_roots((qx64, qy64), (ux64, uy64), (Float64(d_ref.ball_x[j]), Float64(d_ref.ball_y[j])), rad)
            if !isnan(t1) && t1 > 0.0 && t1 < t_min64
                t_min64 = t1; hx64 = r1[1]; hy64 = r1[2]
            end
        end

        err_s = hypot(Float64(d_sim.collision_x) - Float64(d_ref.collision_x),
                      Float64(d_sim.collision_y) - Float64(d_ref.collision_y))
        err_e = hypot(Float64(d_sim.collision_x) - Float64(Float32(hx64)),
                      Float64(d_sim.collision_y) - Float64(Float32(hy64)))

        max_err_simple = max(max_err_simple, err_s)
        sum_sq_simple += err_s^2
        max_err_f64 = max(max_err_f64, err_e)
        sum_sq_f64 += err_e^2

        advance_trajectory_scene!(d_sim, dt_val)
        advance_trajectory_scene!(d_ref, dt_val)
    end

    DESolution.set_algorithmic_dt!(1.0 / 120.0)

    return (
        steps = steps,
        rmse_simple = sqrt(sum_sq_simple / steps),
        max_err_simple = max_err_simple,
        rmse_f64 = sqrt(sum_sq_f64 / steps),
        max_err_f64 = max_err_f64,
        fast_eligible = fast_eligible,
        fast_taken = fast_taken,
        fast_pct = 100.0 * fast_taken / max(1, fast_eligible)
    )
end

function run_all_benchmarks()
    println("=================================================================")
    println("     OPTIMIZED BILLIARD - BENCHMARK SUITE (100,000 RUNS)         ")
    println("=================================================================")

    colors = Dict(
        "Simple"          => PlotUtils.COLOR_SIMPLE,
        "Homotopy"        => PlotUtils.COLOR_HOMOTOPY,
        "DE (Constant)"   => PlotUtils.COLOR_DE_0TH,
        "DE (Euler)"      => PlotUtils.COLOR_DE_EULER,
        "DE (RK4)"        => PlotUtils.COLOR_DE_RK4
    )

    series_names = ["Simple", "DE (Constant)", "DE (Euler)", "DE (RK4)", "Homotopy"]
    dir = @__DIR__

    # -------------------------------------------------------------------------
    # Experiment 1: Scaling with Number of Balls (4, 8, 16, 32, 64)
    # -------------------------------------------------------------------------
    println("\n[1/6] Benchmarking scaling with number of balls (REPETITIONS = $(REPETITIONS))...")
    ball_counts = [4, 8, 16, 32, 64]
    scaling_data = Dict{String, Vector{Float64}}(
        "Simple"        => Float64[],
        "Homotopy"      => Float64[],
        "DE (Constant)" => Float64[],
        "DE (Euler)"    => Float64[],
        "DE (RK4)"      => Float64[]
    )

    for n in ball_counts
        data = create_data(Int32(n); blocked=true)

        t_simple = benchmark_func(SimpleSolution.aim_cue!, data, Int32(1))
        push!(scaling_data["Simple"], t_simple)

        t_homo = benchmark_func(HomotopySolution.aim_cue!, data, Int32(1))
        push!(scaling_data["Homotopy"], t_homo)

        t_de_const = benchmark_func(DEConstantSolution.aim_cue!, data, Int32(1))
        push!(scaling_data["DE (Constant)"], t_de_const)

        t_de_euler = benchmark_func(DEEulerSolution.aim_cue!, data, Int32(1))
        push!(scaling_data["DE (Euler)"], t_de_euler)

        t_de_rk4 = benchmark_func(DERK4Solution.aim_cue!, data, Int32(1))
        push!(scaling_data["DE (RK4)"], t_de_rk4)

        @printf("  N=%2d -> Simple: %6.1f | 0th: %6.1f | Euler: %6.1f | RK4: %6.1f | Homotopy: %6.1f ns\n",
                n, t_simple, t_de_const, t_de_euler, t_de_rk4, t_homo)
    end

    out_scaling = joinpath(dir, "results_ball_scaling.svg")
    PlotUtils.plot_line(
        "Execution Time vs. Number of Balls (100k Runs)",
        "Number of Balls on Table",
        "Latency (ns / simulated update)",
        ball_counts,
        scaling_data,
        colors,
        out_scaling
    )

    # -------------------------------------------------------------------------
    # Experiment 2: Scenario Comparisons (Clear LOS, Blocked LOS, Collisions)
    # -------------------------------------------------------------------------
    println("\n[2/6] Benchmarking diverse operational scenarios (REPETITIONS = $(REPETITIONS))...")
    categories = ["Clear LOS", "Blocked LOS", "Dense (32 Balls)", "Moving LOS"]
    scenario_data = Dict{String, Vector{Float64}}(
        "Simple"        => zeros(Float64, length(categories)),
        "DE (Constant)" => zeros(Float64, length(categories)),
        "DE (Euler)"    => zeros(Float64, length(categories)),
        "DE (RK4)"      => zeros(Float64, length(categories)),
        "Homotopy"      => zeros(Float64, length(categories))
    )

    # Scenario 1: Clear LOS (16 balls, unobstructed initial configuration)
    d_clear = create_data(Int32(16); blocked=false)
    scenario_data["Simple"][1] = benchmark_func(SimpleSolution.aim_cue!, d_clear, Int32(1))
    scenario_data["DE (Constant)"][1] = benchmark_func(DEConstantSolution.aim_cue!, d_clear, Int32(1))
    scenario_data["DE (Euler)"][1] = benchmark_func(DEEulerSolution.aim_cue!, d_clear, Int32(1))
    scenario_data["DE (RK4)"][1] = benchmark_func(DERK4Solution.aim_cue!, d_clear, Int32(1))
    scenario_data["Homotopy"][1] = benchmark_func(HomotopySolution.aim_cue!, d_clear, Int32(1))

    # Scenario 2: Blocked LOS (16 balls, stationary target line with crossing blocker)
    d_blocked = create_data(Int32(16); blocked=true)
    d_blocked.ball_vx[1] = 0.0f0
    d_blocked.ball_vy[1] = 0.0f0
    scenario_data["Simple"][2] = benchmark_func(SimpleSolution.aim_cue!, d_blocked, Int32(1))
    scenario_data["DE (Constant)"][2] = benchmark_func(DEConstantSolution.aim_cue!, d_blocked, Int32(1))
    scenario_data["DE (Euler)"][2] = benchmark_func(DEEulerSolution.aim_cue!, d_blocked, Int32(1))
    scenario_data["DE (RK4)"][2] = benchmark_func(DERK4Solution.aim_cue!, d_blocked, Int32(1))
    scenario_data["Homotopy"][2] = benchmark_func(HomotopySolution.aim_cue!, d_blocked, Int32(1))

    # Scenario 3: Dense Field (32 balls)
    d_dense = create_data(Int32(32); blocked=true)
    scenario_data["Simple"][3] = benchmark_func(SimpleSolution.aim_cue!, d_dense, Int32(1))
    scenario_data["DE (Constant)"][3] = benchmark_func(DEConstantSolution.aim_cue!, d_dense, Int32(1))
    scenario_data["DE (Euler)"][3] = benchmark_func(DEEulerSolution.aim_cue!, d_dense, Int32(1))
    scenario_data["DE (RK4)"][3] = benchmark_func(DERK4Solution.aim_cue!, d_dense, Int32(1))
    scenario_data["Homotopy"][3] = benchmark_func(HomotopySolution.aim_cue!, d_dense, Int32(1))

    # Scenario 4: Moving LOS under the same per-update physics step
    d_moving = create_data(Int32(16); blocked=true)
    scenario_data["Simple"][4] = benchmark_func(SimpleSolution.aim_cue!, d_moving, Int32(1))
    scenario_data["DE (Constant)"][4] = benchmark_func(DEConstantSolution.aim_cue!, d_moving, Int32(1))
    scenario_data["DE (Euler)"][4] = benchmark_func(DEEulerSolution.aim_cue!, d_moving, Int32(1))
    scenario_data["DE (RK4)"][4] = benchmark_func(DERK4Solution.aim_cue!, d_moving, Int32(1))
    scenario_data["Homotopy"][4] = benchmark_func(HomotopySolution.aim_cue!, d_moving, Int32(1))

    for (c_idx, cat) in enumerate(categories)
        @printf("  Scenario '%-18s' -> Simple: %6.1f | 0th: %6.1f | Euler: %6.1f | RK4: %6.1f | Homotopy: %6.1f ns\n",
                cat, scenario_data["Simple"][c_idx], scenario_data["DE (Constant)"][c_idx], scenario_data["DE (Euler)"][c_idx], scenario_data["DE (RK4)"][c_idx], scenario_data["Homotopy"][c_idx])
    end

    out_scenarios = joinpath(dir, "results_scenarios.svg")
    PlotUtils.plot_grouped_bar(
        "Latency Across Operational Scenarios (100k Runs)",
        "Latency (ns / simulated update)",
        categories,
        series_names,
        scenario_data,
        colors,
        out_scenarios
    )

    # -------------------------------------------------------------------------
    # Experiment 3: Simulated-update latency summary (Standard 16-Ball Scene)
    # -------------------------------------------------------------------------
    println("\n[3/6] Generating simulated-update latency summary (REPETITIONS = $(REPETITIONS))...")
    d_std = create_data(Int32(16); blocked=true)
    t_simp = benchmark_func(SimpleSolution.aim_cue!, d_std, Int32(1))
    t_dec  = benchmark_func(DEConstantSolution.aim_cue!, d_std, Int32(1))
    t_dee  = benchmark_func(DEEulerSolution.aim_cue!, d_std, Int32(1))
    t_derk = benchmark_func(DERK4Solution.aim_cue!, d_std, Int32(1))
    t_hom  = benchmark_func(HomotopySolution.aim_cue!, d_std, Int32(1))

    summary_cats = ["Standard 16 Balls"]
    summary_data = Dict{String, Vector{Float64}}(
        "Simple"        => [t_simp],
        "DE (Constant)" => [t_dec],
        "DE (Euler)"    => [t_dee],
        "DE (RK4)"      => [t_derk],
        "Homotopy"      => [t_hom]
    )

    out_summary = joinpath(dir, "results_summary.svg")
    PlotUtils.plot_grouped_bar(
        "Simulated Update Latency (16-Ball Scene, 100k Runs)",
        "Latency (ns / update)",
        summary_cats,
        series_names,
        summary_data,
        colors,
        out_summary
    )

    println("\n-----------------------------------------------------------------")
    println("SIMULATED UPDATE LATENCY (aim + ball physics; rendering excluded):")
    println("-----------------------------------------------------------------")
    @printf("| Solution         | Update (ns) | Relative latency |\n")
    @printf("|:-----------------|------------:|-----------------:|\n")
    @printf("| Simple           | %11.1f | %16.2fx |\n", t_simp, t_simp / t_simp)
    @printf("| DE (Constant)    | %11.1f | %16.2fx |\n", t_dec,  t_dec / t_simp)
    @printf("| DE (Euler)       | %11.1f | %16.2fx |\n", t_dee,  t_dee / t_simp)
    @printf("| DE (RK4)         | %11.1f | %16.2fx |\n", t_derk, t_derk / t_simp)
    @printf("| Homotopy         | %11.1f | %16.2fx |\n", t_hom,  t_hom / t_simp)
    println("-----------------------------------------------------------------")

    # -------------------------------------------------------------------------
    # Experiment 4: Central Loop Fast-Path Frequency & Branch Breakdown
    # -------------------------------------------------------------------------
    println("\n[4/6] Analyzing central loop fast-path hit rates across 100,000 updates...")
    fp_series = ["DE (Constant)", "DE (Euler)", "DE (RK4)"]
    fp_plot_data = Dict{String, Vector{Float64}}(
        "DE (Constant)" => zeros(Float64, length(categories)),
        "DE (Euler)"    => zeros(Float64, length(categories)),
        "DE (RK4)"      => zeros(Float64, length(categories))
    )
    scenario_objs = [d_clear, d_blocked, d_dense, d_moving]

    for (s_idx, (cat, d_scen)) in enumerate(zip(categories, scenario_objs))
        println("  --- Scenario: $cat (100,000 frames) ---")
        for (m_name, m_sym) in (("DE (Constant)", :zero_order), ("DE (Euler)", :euler), ("DE (RK4)", :rk4))
            st = analyze_fastpath_and_loop(d_scen, Int32(1), m_sym; reps=REPETITIONS)
            fp_plot_data[m_name][s_idx] = st.fast_hit_rate_eligible
            @printf("    %-14s | Eligible: %6d (%5.1f%%) | Fast-Path Taken: %6d (%6.2f%% of eligible, %5.2f%% total) | Loop Cont: %5d (Conv: %5d, Fallback: %4d)\n",
                    m_name, st.fast_eligible, 100.0 * st.fast_eligible / st.reps,
                    st.fast_taken, st.fast_hit_rate_eligible, st.fast_hit_rate_all,
                    st.loop_cont_calls, st.loop_cont_conv, st.loop_fallback)
        end
    end

    out_fastpath = joinpath(dir, "results_fastpath.svg")
    PlotUtils.plot_grouped_bar(
        "Fast-Path Hit Rate (% of Blocked Frames, 100k Runs)",
        "Fast-Path Hit Rate (%)",
        categories,
        fp_series,
        fp_plot_data,
        colors,
        out_fastpath
    )

    # -------------------------------------------------------------------------
    # Experiment 5: Very Low Timestep (`dt`) Experiments
    # -------------------------------------------------------------------------
    println("\n[5/6] Running experiments with very small timesteps (low dt)...")
    freq_multipliers = [120, 240, 600, 1200, 2400, 6000, 12000]
    low_dt_fastpath = Dict{String, Vector{Float64}}(
        "DE (Constant)" => Float64[],
        "DE (Euler)"    => Float64[],
        "DE (RK4)"      => Float64[]
    )

    println("  | Rate (Hz) |    dt (s)   | Method        | Fast-Path (%) | RMSE vs F64 (px) | Max Err vs F64 (px) | Latency (ns) |")
    println("  |----------:|------------:|:--------------|--------------:|-----------------:|--------------------:|-------------:|")
    for hz in freq_multipliers
        dt_val = 1.0 / Float64(hz)
        d_dt = create_data(Int32(16); blocked=true, dt_target=dt_val)
        for (m_name, f!, m_sym) in (
            ("DE (Constant)", DEConstantSolution.aim_cue!, :zero_order),
            ("DE (Euler)",    DEEulerSolution.aim_cue!,    :euler),
            ("DE (RK4)",      DERK4Solution.aim_cue!,      :rk4)
        )
            acc = evaluate_trajectory_accuracy(f!, dt_val; mode=m_sym, newton_iters=1, total_time=2.0)
            lat = benchmark_func(f!, d_dt, Int32(1); reps=20_000)
            push!(low_dt_fastpath[m_name], acc.fast_pct)
            @printf("  | %9d | %11.2e | %-13s | %12.2f%% | %16.6f | %19.6f | %12.1f |\n",
                    hz, dt_val, m_name, acc.fast_pct, acc.rmse_f64, acc.max_err_f64, lat)
        end
    end

    out_low_dt = joinpath(dir, "results_low_dt.svg")
    PlotUtils.plot_line(
        "Fast-Path Hit Rate vs. Simulation Frequency (1/dt)",
        "Simulation Frequency 1/dt (Hz)",
        "Fast-Path Hit Rate (%)",
        freq_multipliers,
        low_dt_fastpath,
        colors,
        out_low_dt
    )

    # -------------------------------------------------------------------------
    # Experiment 6: Constant Predictor Newton Iteration Ablation (2 vs 1 vs 0)
    #               & `track_continuation` Validation
    # -------------------------------------------------------------------------
    println("\n[6/6] Validating Constant predictor Newton iteration count (2 vs 1 vs 0) & `track_continuation`...")
    println("  A) Constant Predictor Newton Iterations Ablation (Trajectory Scene, T = 5.0s):")
    println("  |   dt (s)  | Newton Iters | Fast-Path (%) | RMSE vs Simple (px) | Max Err vs Simple (px) | RMSE vs Exact F64 (px) | Latency (ns) |")
    println("  |----------:|-------------:|--------------:|--------------------:|-----------------------:|-----------------------:|-------------:|")
    for dt_val in (1.0/120.0, 1.0/600.0, 1.0/1200.0, 1.0/6000.0)
        d_dt = create_data(Int32(16); blocked=true, dt_target=dt_val)
        for n_iters in (2, 1, 0)
            f_ablation! = n_iters == 1 ? DEConstantSolution.aim_cue! : ((d, s) -> aim_cue_const_ablation!(d, s, Val(n_iters)))
            acc = evaluate_trajectory_accuracy(DEConstantSolution.aim_cue!, dt_val; mode=:zero_order, newton_iters=n_iters, total_time=5.0)
            lat = benchmark_func(f_ablation!, d_dt, Int32(1); reps=20_000)
            @printf("  | %9.2e | %12d | %12.2f%% | %19.6f | %22.6f | %22.6f | %12.1f |\n",
                    dt_val, n_iters, acc.fast_pct, acc.rmse_simple, acc.max_err_simple, acc.rmse_f64, lat)
        end

    end
    DESolution.set_algorithmic_dt!(1.0 / 120.0)

    println("\n  B) `track_continuation` Validation across perpendicular offsets d_perp in [0, R):")
    p_test = (100.0, 200.0); u_test = (1.0, 0.0); n_test = (0.0, -1.0); rad_test = 18.0
    c_start_test = (400.0, 200.0); r_entry_test = (400.0 - rad_test, 200.0)
    println("  | Mode        | Steps | Conv (<1e-4) Rate | Max Pos Error (d_perp <= 15px) | Max Residual ||f(x)||₂ (d_perp=10px) |")
    println("  |:------------|------:|------------------:|-------------------------------:|-------------------------------------:|")
    for m_sym in (:zero_order, :euler, :rk4), st_cnt in (1, 2, 4)
        DESolution.set_predictor_mode!(m_sym)
        conv_cnt = 0
        max_pos_err_15 = 0.0
        res_at_10 = 0.0
        for k in 0:179
            d_perp = k * 0.1
            c_now = (400.0, 200.0 + d_perp)
            exact_pt = DESolution.line_circle_roots(p_test, u_test, c_now, rad_test)[1]
            sol = DESolution.track_continuation(r_entry_test, p_test, u_test, n_test, c_start_test, c_now, rad_test; steps=st_cnt)
            err = DESolution.f_error(sol, p_test, n_test, c_now, rad_test)
            err_sq = err[1]^2 + err[2]^2
            err_sq < 1e-4 && (conv_cnt += 1)
            if d_perp <= 15.0
                max_pos_err_15 = max(max_pos_err_15, hypot(sol[1] - exact_pt[1], sol[2] - exact_pt[2]))
            end
            if k == 100
                res_at_10 = sqrt(err_sq)
            end
        end
        @printf("  | %-11s | %5d | %6d / 180 (%5.1f%%) | %27.3e px | %36.3e |\n",
                string(m_sym), st_cnt, conv_cnt, 100.0 * conv_cnt / 180.0, max_pos_err_15, res_at_10)
    end

    println("\nAll benchmarks, validations, and plots successfully generated in $(dir)!")
end

run_all_benchmarks()

