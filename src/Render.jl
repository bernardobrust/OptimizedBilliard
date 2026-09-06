module Render

using SimpleDirectMediaLayer
using SimpleDirectMediaLayer.LibSDL2

# Precaulculated version of:
# const CIRCLE_R10_OFFSETS = Int32[round(Int32, sqrt(Float32(100 - dy^2))) for dy in -10:10]
const CIRCLE_R10_OFFSETS = Int32[
    0, 4, 6, 7, 8, 8, 9, 9, 10, 10, 10, 10, 10, 9, 9, 8, 8, 7, 6, 4, 0
]

function init_display(width::Int32, height::Int32)
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 16)
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 16)

    @assert SDL_Init(SDL_INIT_EVERYTHING) == 0 "error initializing SDL: $(unsafe_string(SDL_GetError()))"

    win_flags = SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE
    win = SDL_CreateWindow("Optimized Billiard", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, win_flags)
    renderer = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC)

    win, renderer
end

@inline function draw_circle_r10(renderer::Ptr{SDL_Renderer}, cx::Int32, cy::Int32)
    @inbounds for dy in Int32(-10):Int32(10)
        dx = CIRCLE_R10_OFFSETS[dy+11]
        SDL_RenderDrawLine(renderer, cx - dx, cy + dy, cx + dx, cy + dy)
    end
end

function draw_circle(renderer::Ptr{SDL_Renderer}, cx::Int32, cy::Int32, r::Int32)
    if r == 10
        draw_circle_r10(renderer, cx, cy)
        return
    end
    r_sq = r * r
    for dy in (-r):r
        dx = round(Int32, sqrt(Float32(r_sq - dy * dy)))
        SDL_RenderDrawLine(renderer, cx - dx, cy + dy, cx + dx, cy + dy)
    end
end

function render(data)
    SDL_SetRenderDrawColor(data.renderer, 0, 0, 0, 255)
    SDL_RenderClear(data.renderer)

    # Cue
    SDL_SetTextureColorMod(data.cue_texture, 139, 69, 19)
    dst = Ref(SDL_Rect(
        round(Int32, data.cue_x - data.cue_w * 0.5f0),
        round(Int32, data.cue_y - data.cue_h * 0.5f0),
        round(Int32, data.cue_w),
        round(Int32, data.cue_h)
    ))
    center = Ref(SDL_Point(
        round(Int32, data.cue_w * 0.5f0),
        round(Int32, data.cue_h * 0.5f0)
    ))
    SDL_RenderCopyEx(data.renderer, data.cue_texture, C_NULL, dst, Float64(data.cue_angle), center, SDL_FLIP_NONE)

    # Balls
    r = round(Int32, data.ball_radius)
    @inbounds for i in 1:data.num_balls
        if i == data.selected_ball_id
            SDL_SetRenderDrawColor(data.renderer, 0, 120, 255, 255)
        else
            SDL_SetRenderDrawColor(data.renderer, 230, 41, 55, 255)
        end
        draw_circle(data.renderer, round(Int32, data.ball_x[i]), round(Int32, data.ball_y[i]), r)
    end

    SDL_RenderPresent(data.renderer)
end

function close_display(data)
    data.cue_texture != C_NULL && SDL_DestroyTexture(data.cue_texture)
    data.renderer != C_NULL && SDL_DestroyRenderer(data.renderer)
    data.win != C_NULL && SDL_DestroyWindow(data.win)
    SDL_Quit()
end

end