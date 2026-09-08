# Simple solution for cue aiming in a billiards game, based on tracking the contact point on the ball.
# @TODO: Export first collision point of the line of sight (LOS) from cue to ball, and check if any other balls are in the way.

# GEOMETRIC INTERPRETATION OF BALL CONTACT POINT TRACKING AND CUE AIMING
# =============================================================================
# Coordinate Spaces and Kinematics:
# - World/Screen Space: R2 coordinate frame with origin at the top-left,
#   +X pointing right, and +Y pointing down.
# - Ball Rigid Body: Ball i has radius R and center of mass C_i(t) = (ball_x[i], ball_y[i]).
# - Contact Point at Click Time t0:
#   When a user clicks at mouse position M = (mx, my) within the disk ||M - C_i(t0)||_2 ≤ R,
#   we parameterize the contact point in the ball's body-fixed local reference frame:
#       r_offset = M - C_i(t0) = (track_offset_x, track_offset_y)
#   Because the ball undergoes purely translational Newtonian motion without
#   rotational torque (no spin ω), the body-fixed basis does not rotate relative
#   to the world frame. Therefore, r_offset remains constant over time.
# - Dynamic Contact Point at Time t:
#   By affine vector translation:
#       P_track(t) = C_i(t) + r_offset
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
# =============================================================================
@inline function aim_cue!(data, sid::Int32)
    # P_track(t) = C(t) + r_offset
    tx = data.ball_x[sid] + data.track_offset_x
    ty = data.ball_y[sid] + data.track_offset_y

    # V = P_track(t) - Q, θ = atan2(V_y, V_x), cue_angle = θ - 90°
    data.cue_angle = rad2deg(atan(ty - data.cue_y, tx - data.cue_x)) - 90.0f0
end