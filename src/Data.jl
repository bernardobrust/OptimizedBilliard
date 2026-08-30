module Data

using SimpleDirectMediaLayer
using SimpleDirectMediaLayer.LibSDL2

mutable struct DataPlex
    # Delta Time
    last
    freq
    dt_target

    # Square
    x::Float32
    y::Float32
    w::Int32
    h::Int32
    speed::Int32

    # Bounds
    bound_x::Int32
    bound_y::Int32

    # Renderer
    win
    renderer
end

function DataPlex(;
    last,
    freq,
    dt_target,
    x=50.0f0,
    y=50.0f0,
    w=Int32(50),
    h=Int32(50),
    speed=Int32(500),
    bound_x,
    bound_y,
    win,
    renderer
)
    DataPlex(
        last,
        freq,
        dt_target,
        x,
        y,
        w,
        h,
        speed,
        bound_x,
        bound_y,
        win,
        renderer
    )
end

function init_data(
    fps_target::Int32,
    win_w::Int32,
    win_h::Int32,
    win,
    renderer
)::DataPlex
    DataPlex(
        last=SDL_GetPerformanceCounter(),
        freq=SDL_GetPerformanceFrequency(),
        dt_target=1 / fps_target,
        bound_x=win_w,
        bound_y=win_h,
        win=win,
        renderer=renderer
    )
end

end