module Update

include("Input.jl")

# @TODO: Change the solution to based on a toggle (like pressing 's' switches to the next solution)
include("SimpleSolution.jl")

using SimpleDirectMediaLayer
using SimpleDirectMediaLayer.LibSDL2

function update(data)::Bool
    event_ref = Ref{SDL_Event}()
    while Bool(SDL_PollEvent(event_ref))
        evt = event_ref[]

        if evt.type == SDL_QUIT
            return false

        # Workoround for reated space keys
        elseif evt.type == SDL_KEYDOWN
            if evt.key.keysym.scancode == SDL_SCANCODE_SPACE && evt.key.repeat == 0
                data.paused = !data.paused
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

    now = SDL_GetPerformanceCounter()
    dt = Float32((now - data.last) / data.freq)
    data.last = now
    dt = min(dt, 0.05f0)

    rad = data.ball_radius
    max_x = data.bound_x - rad
    max_y = data.bound_y - rad

    # Quit
    Input.is_down(SDL_SCANCODE_ESCAPE) && return false

    # Pause
    Input.is_down(SDL_SCANCODE_SPACE) && (data.paused = !data.paused)

    # We need this interaction to happen even if the game is paused

    # Cue interaction and ball targeting
    mx = Input.mouse_x()
    my = Input.mouse_y()

    # @TODO: Raycast from cue to ball, and check if any other balls are in the way
    if Input.mouse_left_pressed()
        clicked_ball_id = Int32(0)
        rad_sq = rad * rad
        @inbounds for i in 1:data.num_balls
            dx = mx - data.ball_x[i]
            dy = my - data.ball_y[i]
            if dx * dx + dy * dy <= rad_sq
                clicked_ball_id = Int32(i)
                break
            end
        end

        if clicked_ball_id != 0
            data.selected_ball_id = clicked_ball_id
            # Geometric interpretation:
            # Store the local displacement vector from the ball center to the clicked point:
            #   r_offset = M - C = (mx - ball_x[id], my - ball_y[id])
            data.track_offset_x = mx - data.ball_x[clicked_ball_id]
            data.track_offset_y = my - data.ball_y[clicked_ball_id]
            aim_cue!(data, clicked_ball_id)
        end

        # Drag check for rotated cue rectangle
        rad_angle = deg2rad(-data.cue_angle)
        cos_a = cos(rad_angle)
        sin_a = sin(rad_angle)
        dx = mx - data.cue_x
        dy = my - data.cue_y
        local_x = dx * cos_a - dy * sin_a + data.cue_w * 0.5f0
        local_y = dx * sin_a + dy * cos_a + data.cue_h * 0.5f0

        if local_x >= 0.0f0 && local_x <= data.cue_w && local_y >= 0.0f0 && local_y <= data.cue_h
            data.dragging_cue = true
            data.drag_offset_x = mx - data.cue_x
            data.drag_offset_y = my - data.cue_y
        end
    elseif data.selected_ball_id != 0
        aim_cue!(data, data.selected_ball_id)
    end

    if !Input.mouse_left_down()
        data.dragging_cue = false
    elseif data.dragging_cue
        data.cue_x = mx - data.drag_offset_x
        data.cue_y = my - data.drag_offset_y
        if data.selected_ball_id != 0
            aim_cue!(data, data.selected_ball_id)
        end
    end

    # Do not run code after this if the game is paused
    data.paused && (Input.end_frame(); return true)

    # Ball motion and wall reflection
    @inbounds for i in 1:data.num_balls
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

    # Pairwise elastic collisions
    min_dist = 2.0f0 * rad
    min_dist_sq = min_dist * min_dist

    @inbounds for i in 1:(data.num_balls-1)
        x1 = data.ball_x[i]
        y1 = data.ball_y[i]
        vx1 = data.ball_vx[i]
        vy1 = data.ball_vy[i]

        for j in (i+1):data.num_balls
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

                overlap = 0.5f0 * (min_dist - dist)
                x1 -= nx * overlap
                y1 -= ny * overlap
                data.ball_x[j] = x2 + nx * overlap
                data.ball_y[j] = y2 + ny * overlap

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

    # Geometric interpretation (Post-integration re-alignment):
    # Ball positions have updated during this physics step (integration + collisions).
    # Re-evaluating aim_cue! here eliminates any 1-frame latency between the moving ball's
    # tracked contact point and the cue orientation before rendering.
    if !data.paused && data.selected_ball_id != 0
        aim_cue!(data, data.selected_ball_id)
    end

    Input.end_frame()
    true
end

end