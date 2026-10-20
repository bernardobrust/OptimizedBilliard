module Data

using SimpleDirectMediaLayer
using SimpleDirectMediaLayer.LibSDL2

# =============================================================================
# DataPlex: Central mutable game state container.
# Uses a Structure-of-Arrays (SoA) layout for ball positions and velocities
# (`ball_x`, `ball_y`, `ball_vx`, `ball_vy`) to maximize cache locality and
# enable SIMD vectorization during physics integration and raycasting loops.
# =============================================================================
mutable struct DataPlex
    # High-resolution timing state (SDL performance counter)
    last::UInt64          # Timestamp counter at the previous frame
    freq::UInt64          # Hardware counter ticks per second
    dt_target::Float64    # Target fixed timestep (seconds), e.g., 1/120 s

    # Table boundary dimensions (pixels)
    bound_x::Float32      # Table width (right wall x-coordinate)
    bound_y::Float32      # Table height (bottom wall y-coordinate)

    # Balls (Structure of Arrays for SIMD-friendly memory layout)
    num_balls::Int32      # Total number of balls on the table
    ball_radius::Float32  # Uniform radius R of each ball (pixels)
    ball_x::Vector{Float32}  # X-coordinates of ball centers
    ball_y::Vector{Float32}  # Y-coordinates of ball centers
    ball_vx::Vector{Float32} # X-components of ball velocities (px/s)
    ball_vy::Vector{Float32} # Y-components of ball velocities (px/s)

    # Cue stick geometry, interaction, and Line-of-Sight (LOS) tracking state
    cue_x::Float32           # Cue pivot X position (ray origin qx)
    cue_y::Float32           # Cue pivot Y position (ray origin qy)
    cue_w::Float32           # Cue visual width (pixels)
    cue_h::Float32           # Cue visual length (pixels)
    cue_angle::Float32       # Cue rotation angle in degrees for SDL rendering
    dragging_cue::Bool       # True while the user is dragging the cue stick
    drag_offset_x::Float32   # Mouse-to-cue-pivot X offset during drag
    drag_offset_y::Float32   # Mouse-to-cue-pivot Y offset during drag
    selected_ball_id::Int32  # Index of tracked target ball (0 = fixed-direction aim)
    track_offset_x::Float32  # Local body-fixed X offset from target ball center
    track_offset_y::Float32  # Local body-fixed Y offset from target ball center
    collision_x::Float32     # Computed first-hit X coordinate along the LOS ray
    collision_y::Float32     # Computed first-hit Y coordinate along the LOS ray
    target_x::Float32        # Last clicked world X target coordinate
    target_y::Float32        # Last clicked world Y target coordinate
    aim_dir_x::Float32       # Normalized X direction when aiming at empty space (sid == 0)
    aim_dir_y::Float32       # Normalized Y direction when aiming at empty space (sid == 0)
    has_aim::Bool            # True once the user has clicked to establish an aim ray

    # SDL2 Window, Renderer & GPU Texture resources
    win::Ptr{SDL_Window}
    renderer::Ptr{SDL_Renderer}
    cue_texture::Ptr{SDL_Texture}

    # Simulation pause flag (toggled with Spacebar)
    paused::Bool

    # Active aiming solver (:simple, :homotopy, :de_constant, :de_euler, or :de_rk4)
    sol_id::Int32
    current_solution::Symbol

    # HUD Font & Rolling FPS Counter (instantaneous EMA + 10-second ring buffer)
    font::Ptr{TTF_Font}
    fps::Float64                      # Exponentially smoothed instantaneous FPS
    fps_10s::Float64                  # Rolling average FPS over the last 10 seconds
    frame_timestamps::Vector{Float64} # Circular buffer of frame timestamps (seconds)
    frame_head::Int                   # Ring buffer head index (oldest frame in 10s window)
    frame_tail::Int                   # Ring buffer tail index (next write slot)
    frame_capacity::Int               # Maximum capacity of the timestamp ring buffer
end

"""
    init_data(fps_target, win_w, win_h, win, renderer, font) -> DataPlex

Allocates and initializes the `DataPlex` state with `num_balls = 16` balls placed
randomly inside the table bounds with uniform initial speed and random directions.
"""
function init_data(
    fps_target::Int32,
    win_w::Int32,
    win_h::Int32,
    win::Ptr{SDL_Window},
    renderer::Ptr{SDL_Renderer},
    font::Ptr{TTF_Font} = Ptr{TTF_Font}(C_NULL)
)::DataPlex
    num_balls = Int32(16)
    ball_radius = 18.0f0
    speed = 140.0f0

    ball_x = Vector{Float32}(undef, num_balls)
    ball_y = Vector{Float32}(undef, num_balls)
    ball_vx = Vector{Float32}(undef, num_balls)
    ball_vy = Vector{Float32}(undef, num_balls)

    # Randomly initialize ball positions within [R, bound - R] and velocities of magnitude `speed`
    for i in 1:num_balls
        ball_x[i] = ball_radius + rand(Float32) * (Float32(win_w) - 2.0f0 * ball_radius)
        ball_y[i] = ball_radius + rand(Float32) * (Float32(win_h) - 2.0f0 * ball_radius)

        angle = rand(Float32) * (2.0f0 * Float32(pi))
        ball_vx[i] = cos(angle) * speed
        ball_vy[i] = sin(angle) * speed
    end

    # Create a 1x1 white static texture used by SDL_RenderCopyEx to draw the rotated cue rectangle
    cue_texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STATIC, 1, 1)
    pixel = UInt32[0xFFFFFFFF]
    SDL_UpdateTexture(cue_texture, C_NULL, pixel, sizeof(UInt32))

    # Ring buffer for computing the 10-second amortized FPS metric
    frame_capacity = 65536
    frame_timestamps = zeros(Float64, frame_capacity)

    DataPlex(
        SDL_GetPerformanceCounter(),
        SDL_GetPerformanceFrequency(),
        1.0 / Float64(fps_target),
        Float32(win_w),
        Float32(win_h),
        num_balls,
        ball_radius,
        ball_x,
        ball_y,
        ball_vx,
        ball_vy,
        100.0f0,
        200.0f0,
        16.0f0,
        220.0f0,
        30.0f0,
        false,
        0.0f0,
        0.0f0,
        Int32(0),
        0.0f0,
        0.0f0,
        0.0f0,
        0.0f0,
        0.0f0,
        0.0f0,
        0.0f0,
        0.0f0,
        false,
        win,
        renderer,
        cue_texture,
        false,
        0,
        :simple,
        font,
        0.0,
        0.0,
        frame_timestamps,
        1,
        1,
        frame_capacity
    )
end

end

