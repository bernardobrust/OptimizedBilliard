# Simple solution for cue aiming in a billiards game, based on tracking the contact point on the ball
# and raycasting the Line of Sight (LOS) against all other balls to find the first collision point.

# GEOMETRIC INTERPRETATION OF BALL CONTACT POINT TRACKING AND CUE AIMING
# =============================================================================
# Coordinate Spaces and Kinematics:
# - World/Screen Space: R² coordinate frame with origin at the top-left,
#   +X pointing right, and +Y pointing down.
# - Ball Rigid Body: Ball i has radius R and center of mass Cᵢ(t) = (ball_x[i], ball_y[i]).
# - Contact Point at Click Time t₀:
#   When a user clicks at mouse position M = (mx, my) within the disk ||M - Cᵢ(t₀)||₂ ≤ R,
#   we parameterize the contact point in the ball's body-fixed local reference frame:
#       r_offset = M - Cᵢ(t₀) = (track_offset_x, track_offset_y)
#   Because the ball undergoes purely translational Newtonian motion without
#   rotational torque (no spin ω), the body-fixed basis does not rotate relative
#   to the world frame. Therefore, r_offset remains constant over time.
# - Dynamic Contact Point at Time t:
#   By affine vector translation:
#       P_track(t) = Cᵢ(t) + r_offset
#                  = (ball_x[i](t) + track_offset_x, ball_y[i](t) + track_offset_y)
#
# Sightline Vector and Cue Orientation:
# - Cue Position: Let Q = (cue_x, cue_y) be the center/pivot of the cue stick.
# - Sightline Vector: The ray connecting the cue pivot to the target contact point is:
#       V(t) = P_track(t) - Q = (P_track_x - cue_x, P_track_y - cue_y)
# - Direction Angle: In R2, the polar azimuth angle of V(t) relative to the +X axis is:
#       θ = atan2(V_y, V_x) = atan(V_y, V_x) [in radians]
# - SDL Orientation Alignment:
#   The cue sprite texture is unrotated at angle 0°, where its length extends along
#   the +Y axis (downward, i.e., +90° in standard polar coords).
#   To rotate the cue so that its tip aims directly along V(t) toward P_track(t):
#       cue_angle = rad2deg(θ) - 90°
#
# LOS Line-Circle Collision Check:
# - The LOS is a line segment starting at Q and aiming along the normalized direction:
#       u = V(t) / ||V(t)||₂
#   with target distance L = ||V(t)||₂.
# - For every other ball j ≠ sid with center Cⱼ = (ball_x[j], ball_y[j]) and radius R:
#   1. Vector from ray origin to ball center: Wⱼ = Cⱼ - Q.
#   2. Scalar projection onto ray: t_proj = Wⱼ · u.
#      If t_proj ≤ -R or t_proj ≥ t_min + R, ball j cannot block the ray earlier than
#      the current closest hit t_min (where initially t_min = L).
#   3. Orthogonal distance squared from Cⱼ to the ray line:
#          d_perp² = ||Wⱼ||² - t_proj²
#   4. If d_perp² < R², the ray intersects circle j. The half-chord length is:
#          h = sqrt(R² - d_perp²)
#      The entry point distance along the ray is t_entry = t_proj - h.
#   5. If 0 < t_entry < t_min, ball j is directly between the cue and the target,
#      blocking the line of sight! We update t_min = t_entry and the collision point:
#          P_collision = Q + t_min * u
# - If no ball blocks the LOS, the ray reaches the target unobstructed:
#          P_collision = P_track(t)
# - The resulting (collision_x, collision_y) is exported to data for rendering.
# =============================================================================

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

    # Raycasting hit
    @inbounds for j in 1:data.num_balls
        j == sid && continue

        vx = data.ball_x[j] - qx
        vy = data.ball_y[j] - qy

        t_proj = vx * ux + vy * uy

        # Early rejection: ball is behind cue origin or past current closest collision
        if t_proj <= -rad || t_proj >= t_min + rad
            continue
        end

        d_perp_sq = (vx * vx + vy * vy) - t_proj * t_proj
        if d_perp_sq < rad_sq
            h = sqrt(rad_sq - d_perp_sq)
            t_entry = t_proj - h
            if t_entry > 0.0f0 && t_entry < t_min
                t_min = t_entry
                hit_x = qx + t_entry * ux
                hit_y = qy + t_entry * uy
            end
        end
    end

    data.collision_x = hit_x
    data.collision_y = hit_y
end
