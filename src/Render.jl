module Render

using SimpleDirectMediaLayer
using SimpleDirectMediaLayer.LibSDL2

# Precaulculated version of:
# const CIRCLE_R10_OFFSETS = Int32[round(Int32, sqrt(Float32(100 - dy^2))) for dy in -10:10]
const CIRCLE_R10_OFFSETS = Int32[
    0, 4, 6, 7, 8, 8, 9, 9, 10, 10, 10, 10, 10, 9, 9, 8, 8, 7, 6, 4, 0
]

@inline function init_display(width::Int32, height::Int32)
    @assert SDL_Init(SDL_INIT_EVERYTHING) == 0 "error initializing SDL: $(unsafe_string(SDL_GetError()))"
    @assert TTF_Init() == 0 "error initializing TTF: $(unsafe_string(SDL_GetError()))"

    win_flags = SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE
    win = SDL_CreateWindow("Optimized Billiard", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, win_flags)
    renderer = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED)

    win, renderer
end

function load_font(ptsize::Int32 = Int32(16))::Ptr{TTF_Font}
    candidates = String[]
    try
        p = normpath(joinpath(dirname(dirname(pathof(SimpleDirectMediaLayer))), "assets", "fonts", "FiraCode", "ttf", "FiraCode-Regular.ttf"))
        push!(candidates, p)
    catch
    end
    if Sys.iswindows()
        windir = get(ENV, "WINDIR", "C:\\Windows")
        push!(candidates, joinpath(windir, "Fonts", "arial.ttf"))
        push!(candidates, joinpath(windir, "Fonts", "consola.ttf"))
        push!(candidates, joinpath(windir, "Fonts", "segoeui.ttf"))
        push!(candidates, joinpath(windir, "Fonts", "tahoma.ttf"))
    elseif Sys.isapple()
        push!(candidates, "/Library/Fonts/Arial.ttf")
        push!(candidates, "/System/Library/Fonts/SFNSMono.ttf")
    else
        push!(candidates, "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
        push!(candidates, "/usr/share/fonts/TTF/DejaVuSans.ttf")
    end
    for path in candidates
        if isfile(path)
            font = TTF_OpenFont(path, ptsize)
            if font != C_NULL
                return font
            end
        end
    end
    @warn "No TTF font could be loaded from candidates: $(candidates)"
    return Ptr{TTF_Font}(C_NULL)
end

@inline function draw_circle_r10(renderer::Ptr{SDL_Renderer}, cx::Int32, cy::Int32)
    @inbounds for dy in Int32(-10):Int32(10)
        dx = CIRCLE_R10_OFFSETS[dy+11]
        SDL_RenderDrawLine(renderer, cx - dx, cy + dy, cx + dx, cy + dy)
    end
end

@inline function draw_circle(renderer::Ptr{SDL_Renderer}, cx::Int32, cy::Int32, r::Int32)
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

@inline function render(data)
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

    # Tracking indicator for the exact point on the selected ball
    if data.selected_ball_id != 0
        sid = data.selected_ball_id
        # Exact world coordinates of the tracked contact point on the target ball
        tx = round(Int32, data.ball_x[sid] + data.track_offset_x)
        ty = round(Int32, data.ball_y[sid] + data.track_offset_y)

        # First collision point of the LOS (exported by the solution)
        cx = round(Int32, data.collision_x)
        cy = round(Int32, data.collision_y)

        # LOS from cue center to the first collision point (stops at blocking ball)
        SDL_SetRenderDrawColor(data.renderer, 255, 215, 0, 160)
        SDL_RenderDrawLine(data.renderer, round(Int32, data.cue_x), round(Int32, data.cue_y), cx, cy)

        # Reticle indicator at the exact tracked point on the target ball
        SDL_SetRenderDrawColor(data.renderer, 255, 255, 255, 255)
        SDL_RenderDrawLine(data.renderer, tx - 3, ty, tx + 3, ty)
        SDL_RenderDrawLine(data.renderer, tx, ty - 3, tx, ty + 3)

        SDL_SetRenderDrawColor(data.renderer, 255, 215, 0, 255)
        SDL_RenderDrawPoint(data.renderer, tx, ty)

        # If LOS is blocked before reaching the target, draw a collision impact indicator
        if (data.collision_x - (data.ball_x[sid] + data.track_offset_x))^2 +
            (data.collision_y - (data.ball_y[sid] + data.track_offset_y))^2 > 1.0f0
            SDL_SetRenderDrawColor(data.renderer, 0, 255, 0, 255)
            SDL_RenderDrawLine(data.renderer, cx - 2, cy, cx + 2, cy)
            SDL_RenderDrawLine(data.renderer, cx, cy - 2, cx, cy + 2)
        end
    elseif data.has_aim
        # First collision point of the LOS (exported by the solution)
        cx = round(Int32, data.collision_x)
        cy = round(Int32, data.collision_y)

        # LOS from cue center to the first collision point (stops at blocking ball or boundary)
        SDL_SetRenderDrawColor(data.renderer, 255, 215, 0, 160)
        SDL_RenderDrawLine(data.renderer, round(Int32, data.cue_x), round(Int32, data.cue_y), cx, cy)

        # Reticle indicator at the collision point
        SDL_SetRenderDrawColor(data.renderer, 255, 255, 255, 255)
        SDL_RenderDrawLine(data.renderer, cx - 3, cy, cx + 3, cy)
        SDL_RenderDrawLine(data.renderer, cx, cy - 3, cx, cy + 3)

        SDL_SetRenderDrawColor(data.renderer, 255, 215, 0, 255)
        SDL_RenderDrawPoint(data.renderer, cx, cy)
    end

    # FPS counter in top-right
    render_fps(data)

    SDL_RenderPresent(data.renderer)
end

@inline function render_fps(data)
    if data.font == C_NULL
        data.font = load_font(Int32(16))
        data.font == C_NULL && return
    end

    fps_str = "FPS: $(round(Int, data.fps))"
    fps_10s_str = "10s: $(round(data.fps_10s, digits=1))"

    w_ref = Ref{Cint}(0)
    h_ref = Ref{Cint}(0)
    SDL_GetRendererOutputSize(data.renderer, w_ref, h_ref)
    win_w = w_ref[] > 0 ? w_ref[] : round(Int32, data.bound_x)

    color = SDL_Color(255, 255, 255, 255)
    margin = Int32(12)
    spacing = Int32(3)

    surf1 = TTF_RenderText_Blended(data.font, fps_str, color)
    surf2 = TTF_RenderText_Blended(data.font, fps_10s_str, color)

    surf1 == C_NULL && surf2 == C_NULL && return

    fw1, fh1 = Ref{Cint}(0), Ref{Cint}(0)
    fw2, fh2 = Ref{Cint}(0), Ref{Cint}(0)

    surf1 != C_NULL && TTF_SizeText(data.font, fps_str, fw1, fh1)
    surf2 != C_NULL && TTF_SizeText(data.font, fps_10s_str, fw2, fh2)

    max_w = max(fw1[], fw2[])
    total_h = fh1[] + (surf2 != C_NULL ? fh2[] + spacing : Int32(0))

    pad = Int32(6)
    bg_rect = Ref(SDL_Rect(win_w - max_w - margin - pad, margin - pad, max_w + 2 * pad, total_h + 2 * pad))
    SDL_SetRenderDrawBlendMode(data.renderer, SDL_BLENDMODE_BLEND)
    SDL_SetRenderDrawColor(data.renderer, 0, 0, 0, 160)
    SDL_RenderFillRect(data.renderer, bg_rect)

    if surf1 != C_NULL
        tex1 = SDL_CreateTextureFromSurface(data.renderer, surf1)
        if tex1 != C_NULL
            dst1 = Ref(SDL_Rect(win_w - fw1[] - margin, margin, fw1[], fh1[]))
            SDL_RenderCopy(data.renderer, tex1, C_NULL, dst1)
            SDL_DestroyTexture(tex1)
        end
        SDL_FreeSurface(surf1)
    end

    if surf2 != C_NULL
        tex2 = SDL_CreateTextureFromSurface(data.renderer, surf2)
        if tex2 != C_NULL
            dst2 = Ref(SDL_Rect(win_w - fw2[] - margin, margin + fh1[] + spacing, fw2[], fh2[]))
            SDL_RenderCopy(data.renderer, tex2, C_NULL, dst2)
            SDL_DestroyTexture(tex2)
        end
        SDL_FreeSurface(surf2)
    end
end

@inline function close_display(data)
    data.font != C_NULL && TTF_CloseFont(data.font)
    TTF_Quit()
    data.cue_texture != C_NULL && SDL_DestroyTexture(data.cue_texture)
    data.renderer != C_NULL && SDL_DestroyRenderer(data.renderer)
    data.win != C_NULL && SDL_DestroyWindow(data.win)
    SDL_Quit()
end

end
