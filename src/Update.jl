module Update

# =============================================================================
# Update Module
# Handles SDL event polling, FPS measurement (instantaneous EMA + 10s rolling
# window), mouse/cue interaction, ball kinematics with wall reflections,
# pairwise elastic ball-ball collisions, and fixed-rate aiming dispatch.
# =============================================================================

include("Input.jl")

include("SimpleSolution.jl")
include("HomotopySolution.jl")
include("DESolution.jl")
include("DEConstantSolution.jl")
include("DEEulerSolution.jl")
include("DERK4Solution.jl")

using SimpleDirectMediaLayer
using SimpleDirectMediaLayer.LibSDL2

# Fixed algorithmic time step (120 Hz) for aiming predictor-corrector updates
const AIM_ALGORITHM_DT = 1.0 / 120.0
const AIM_ACCUMULATOR = Ref(0.0)

# Dispatches the aiming calculation to the currently active solver module
@inline function aim_cue!(data, sid::Int32)
    DESolution.set_algorithmic_dt!(AIM_ALGORITHM_DT)
    if data.current_solution == :simple
        SimpleSolution.aim_cue!(data, sid)
    elseif data.current_solution == :homotopy
        HomotopySolution.aim_cue!(data, sid)
    elseif data.current_solution == :de_constant
        DEConstantSolution.aim_cue!(data, sid)
    elseif data.current_solution == :de_euler
        DEEulerSolution.aim_cue!(data, sid)
    else
        DERK4Solution.aim_cue!(data, sid)
    end
end

# Runs the aiming solver at a fixed 120 Hz cadence using a time accumulator
@inline function update_aim_at_fixed_rate!(data, elapsed::Float32)
    if data.selected_ball_id == 0 && !data.has_aim
        AIM_ACCUMULATOR[] = 0.0
        return
    end

    AIM_ACCUMULATOR[] += Float64(elapsed)
    while AIM_ACCUMULATOR[] >= AIM_ALGORITHM_DT
        aim_cue!(data, data.selected_ball_id)
        AIM_ACCUMULATOR[] -= AIM_ALGORITHM_DT
    end
end

@inline function update(data)::Bool
    # 1. Poll all pending SDL input and window events
    event_ref = Ref{SDL_Event}()
    while Bool(SDL_PollEvent(event_ref))
        evt = event_ref[]

        if evt.type == SDL_QUIT
            return false

            # Workaround for repeated space keys
        elseif evt.type == SDL_KEYDOWN
            if evt.key.keysym.scancode == SDL_SCANCODE_SPACE && evt.key.repeat == 0
                data.paused = !data.paused
            elseif evt.key.keysym.scancode == SDL_SCANCODE_S && evt.key.repeat == 0
                # Cycle through the 5 aiming solvers: Simple -> Homotopy -> DE Constant -> DE Euler -> DE RK4
                data.sol_id += 1
                data.sol_id %= 5
                if data.sol_id == 0
                    data.current_solution = :simple
                elseif data.sol_id == 1
                    data.current_solution = :homotopy
                elseif data.sol_id == 2
                    data.current_solution = :de_constant
                    DESolution.set_predictor_mode!(:zero_order)
                elseif data.sol_id == 3
                    data.current_solution = :de_euler
                    DESolution.set_predictor_mode!(:euler)
                elseif data.sol_id == 4
                    data.current_solution = :de_rk4
                    DESolution.set_predictor_mode!(:rk4)
                end

            else
                Input.handle_key(true, evt.key.keysym.scancode)
            end

        elseif evt.type == SDL_KEYUP
            Input.handle_key(false, evt.key.keysym.scancode)

        elseif evt.type == SDL_MOUSEBUTTONDOWN
            Input.handle_mouse_button(true, evt.button.button, evt.button.x, evt.button.y)

        elseif evt.type == SDL_MOUSEBUTTONUP
            Input.handle_mouse_button(false, evt.button.button, evt.button.x, evt.button.y)

        elseif evt.type == SDL_MOUSEMOTION
            Input.handle_mouse_motion(evt.motion.x, evt.motion.y)
        end
    end

    # 2. Measure elapsed wall-clock frame time `dt` using high-resolution counter
    now = SDL_GetPerformanceCounter()
    dt = Float32((now - data.last) / data.freq)
    data.last = now

    # Calculate Current FPS (Exponential Moving Average with α = 0.05)
    raw_dt = Float64(dt)
    if raw_dt > 0.0
        instant_fps = 1.0 / raw_dt
        data.fps = data.fps == 0.0 ? instant_fps : (0.95 * data.fps + 0.05 * instant_fps)
    end

    # Calculate 10-second Amortized FPS using circular timestamp buffer
    curr_time = Float64(now) / Float64(data.freq)
    data.frame_timestamps[data.frame_tail] = curr_time
    data.frame_tail = (data.frame_tail % data.frame_capacity) + 1
    if data.frame_tail == data.frame_head
        data.frame_head = (data.frame_head % data.frame_capacity) + 1
    end

    # Evict frame timestamps older than 10.0 seconds from the head of the ring buffer
    while data.frame_head != data.frame_tail && (curr_time - data.frame_timestamps[data.frame_head]) > 10.0
        data.frame_head = (data.frame_head % data.frame_capacity) + 1
    end

    num_frames = (data.frame_tail - data.frame_head + data.frame_capacity) % data.frame_capacity
    if num_frames > 1
        time_span = curr_time - data.frame_timestamps[data.frame_head]
        if time_span > 0.0
            data.fps_10s = Float64(num_frames - 1) / time_span
        end
    else
        data.fps_10s = data.fps
    end

    # Clamp physics `dt` to at most 50 ms to prevent tunneling during window drags or stalls
    dt = min(dt, 0.05f0)

    rad = data.ball_radius
    max_x = data.bound_x - rad
    max_y = data.bound_y - rad

    # Quit when Escape is pressed
    Input.is_down(SDL_SCANCODE_ESCAPE) && return false

    # We need this interaction to happen even if the game is paused

    # 3. Cue interaction and ball targeting
    mx = Input.mouse_x()
    my = Input.mouse_y()

    if Input.mouse_left_pressed()
        # Transform mouse coordinates into the cue's local unrotated bounding box frame
        rad_angle = deg2rad(-data.cue_angle)
        cos_a = cos(rad_angle)
        sin_a = sin(rad_angle)
        dx = mx - data.cue_x
        dy = my - data.cue_y
        local_x = dx * cos_a - dy * sin_a + data.cue_w * 0.5f0
        local_y = dx * sin_a + dy * cos_a + data.cue_h * 0.5f0

        # If click lands inside the cue sprite, initiate cue dragging
        if local_x >= 0.0f0 && local_x <= data.cue_w && local_y >= 0.0f0 && local_y <= data.cue_h
            data.dragging_cue = true
            data.drag_offset_x = mx - data.cue_x
            data.drag_offset_y = my - data.cue_y
        else
            # Check if click lands inside any ball disk ||M - Cᵢ||₂² ≤ R²
            clicked_ball_id = Int32(0)
            rad_sq = rad * rad
            @inbounds for i in 1:data.num_balls
                bx = mx - data.ball_x[i]
                by = my - data.ball_y[i]
                if bx * bx + by * by <= rad_sq
                    clicked_ball_id = Int32(i)
                    break
                end
            end

            # Start tracking the clicked contact offset on ball `clicked_ball_id`
            if clicked_ball_id != 0
                data.selected_ball_id = clicked_ball_id
                data.track_offset_x = mx - data.ball_x[clicked_ball_id]
                data.track_offset_y = my - data.ball_y[clicked_ball_id]
                data.target_x = mx
                data.target_y = my
                data.has_aim = true
            else
                # Fixed direction clicked in empty table space
                cdx = mx - data.cue_x
                cdy = my - data.cue_y
                cdist = sqrt(cdx * cdx + cdy * cdy)
                if cdist > 1.0f-6
                    data.aim_dir_x = cdx / cdist
                    data.aim_dir_y = cdy / cdist
                    data.target_x = mx
                    data.target_y = my
                    data.track_offset_x = 0.0f0
                    data.track_offset_y = 0.0f0
                    data.selected_ball_id = Int32(0)
                    data.has_aim = true
                end
            end
        end
    end

    if !Input.mouse_left_down()
        data.dragging_cue = false
    elseif data.dragging_cue
        data.cue_x = mx - data.drag_offset_x
        data.cue_y = my - data.drag_offset_y
    end

    # Do not advance ball physics if the simulation is paused, but still update aiming
    if data.paused
        update_aim_at_fixed_rate!(data, Float32(raw_dt))
        Input.end_frame()
        return true
    end

    # 4. Ball linear motion integration and specular wall reflection
    @inbounds @fastmath @simd for i in 1:data.num_balls
        x = data.ball_x[i] + data.ball_vx[i] * dt
        y = data.ball_y[i] + data.ball_vy[i] * dt
        vx = data.ball_vx[i]
        vy = data.ball_vy[i]

        if x < rad
            x = rad
            vx = -vx
        elseif x > max_x
            x = max_x
            vx = -vx
        end

        if y < rad
            y = rad
            vy = -vy
        elseif y > max_y
            y = max_y
            vy = -vy
        end

        data.ball_x[i] = x
        data.ball_y[i] = y
        data.ball_vx[i] = vx
        data.ball_vy[i] = vy
    end

    # 5. Pairwise equal-mass elastic ball-ball collision resolution (O(N²))
    min_dist = 2.0f0 * rad
    min_dist_sq = min_dist * min_dist

    @inbounds for i in 1:(data.num_balls-1)
        x1 = data.ball_x[i]
        y1 = data.ball_y[i]
        vx1 = data.ball_vx[i]
        vy1 = data.ball_vy[i]

        @inbounds @fastmath @simd for j in (i+1):data.num_balls
            x2 = data.ball_x[j]
            y2 = data.ball_y[j]
            dx = x2 - x1
            dy = y2 - y1
            dist_sq = dx * dx + dy * dy

            if dist_sq < min_dist_sq
                dist = sqrt(dist_sq)
                if dist == 0.0f0
                    nx = 1.0f0
                    ny = 0.0f0
                else
                    inv_dist = 1.0f0 / dist
                    nx = dx * inv_dist
                    ny = dy * inv_dist
                end

                # Positional depenetration: separate overlapping balls equally along normal n
                overlap = 0.5f0 * (min_dist - dist)
                x1 -= nx * overlap
                y1 -= ny * overlap
                data.ball_x[j] = x2 + nx * overlap
                data.ball_y[j] = y2 + ny * overlap

                # Impulse exchange along collision normal for approaching balls (rel_speed > 0)
                vx2 = data.ball_vx[j]
                vy2 = data.ball_vy[j]
                rel_speed = (vx1 - vx2) * nx + (vy1 - vy2) * ny
                if rel_speed > 0.0f0
                    vx1 -= rel_speed * nx
                    vy1 -= rel_speed * ny
                    data.ball_vx[j] = vx2 + rel_speed * nx
                    data.ball_vy[j] = vy2 + rel_speed * ny
                end
            end
        end

        data.ball_x[i] = x1
        data.ball_y[i] = y1
        data.ball_vx[i] = vx1
        data.ball_vy[i] = vy1
    end

    # 6. Aiming has its own fixed 120 Hz cadence; physics, input, and rendering remain
    # tied to the frame rate.
    update_aim_at_fixed_rate!(data, Float32(raw_dt))

    Input.end_frame()
    true
end

end

