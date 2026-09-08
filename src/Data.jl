module Data

using SimpleDirectMediaLayer
using SimpleDirectMediaLayer.LibSDL2

mutable struct DataPlex
    # Delta Time
    last::UInt64
    freq::UInt64
    dt_target::Float64

    # Bounds
    bound_x::Float32
    bound_y::Float32

    # Balls (Structure of Arrays)
    num_balls::Int32
    ball_radius::Float32
    ball_x::Vector{Float32}
    ball_y::Vector{Float32}
    ball_vx::Vector{Float32}
    ball_vy::Vector{Float32}

    # Cue
    cue_x::Float32
    cue_y::Float32
    cue_w::Float32
    cue_h::Float32
    cue_angle::Float32
    dragging_cue::Bool
    drag_offset_x::Float32
    drag_offset_y::Float32
    selected_ball_id::Int32
    track_offset_x::Float32
    track_offset_y::Float32
    collision_x::Float32
    collision_y::Float32

    # Renderer & Resources
    win::Ptr{SDL_Window}
    renderer::Ptr{SDL_Renderer}
    cue_texture::Ptr{SDL_Texture}

    # Pause system
    paused::Bool
end

function init_data(
    fps_target::Int32,
    win_w::Int32,
    win_h::Int32,
    win::Ptr{SDL_Window},
    renderer::Ptr{SDL_Renderer}
)::DataPlex
    num_balls = Int32(16)
    ball_radius = 10.0f0
    speed = 140.0f0

    ball_x = Vector{Float32}(undef, num_balls)
    ball_y = Vector{Float32}(undef, num_balls)
    ball_vx = Vector{Float32}(undef, num_balls)
    ball_vy = Vector{Float32}(undef, num_balls)

    for i in 1:num_balls
        ball_x[i] = ball_radius + rand(Float32) * (Float32(win_w) - 2.0f0 * ball_radius)
        ball_y[i] = ball_radius + rand(Float32) * (Float32(win_h) - 2.0f0 * ball_radius)

        angle = rand(Float32) * (2.0f0 * Float32(pi))
        ball_vx[i] = cos(angle) * speed
        ball_vy[i] = sin(angle) * speed
    end

    # 1x1 white texture for hardware-accelerated rotated rendering
    cue_texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STATIC, 1, 1)
    pixel = UInt32[0xFFFFFFFF]
    SDL_UpdateTexture(cue_texture, C_NULL, pixel, sizeof(UInt32))

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
        10.0f0,
        180.0f0,
        30.0f0,
        false,
        0.0f0,
        0.0f0,
        Int32(0),
        0.0f0,
        0.0f0,
        0.0f0,
        0.0f0,
        win,
        renderer,
        cue_texture,
        false
    )
end

end