module Render

using SimpleDirectMediaLayer
using SimpleDirectMediaLayer.LibSDL2

function init_display(width::Int32, height::Int32)
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 16)
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 16)

    @assert SDL_Init(SDL_INIT_EVERYTHING) == 0 "error initializing SDL: $(unsafe_string(SDL_GetError()))"

    win_flags = SDL_WINDOW_VULKAN | SDL_WINDOW_BORDERLESS | SDL_WINDOW_RESIZABLE
    win = SDL_CreateWindow("Optimized Billiard", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, win_flags)

    renderer = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC)

    win, renderer
end

function render(data)
    SDL_SetRenderDrawColor(data.renderer, 255, 255, 255, 255)
    rect = Ref(SDL_Rect(
        round(Int, data.x),
        round(Int, data.y),
        data.w,
        data.h
    ))
    SDL_RenderFillRect(data.renderer, rect)

    SDL_RenderPresent(data.renderer)

    SDL_SetRenderDrawColor(data.renderer, 0, 0, 0, 255)
    SDL_RenderClear(data.renderer)
end

end