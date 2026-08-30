# Example from the GH page for SDL with Julia, adapted

using SimpleDirectMediaLayer
using SimpleDirectMediaLayer.LibSDL2

dt_target::Float64 = 1 / 120

SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 16)
SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 16)

@assert SDL_Init(SDL_INIT_EVERYTHING) == 0 "error initializing SDL: $(unsafe_string(SDL_GetError()))"

win_flags = SDL_WINDOW_VULKAN | SDL_WINDOW_BORDERLESS | SDL_WINDOW_RESIZABLE
win_w::Int64 = 800
win_h::Int64 = 450
win = SDL_CreateWindow("Game", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, win_w, win_h, win_flags)

renderer = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC)

surface = IMG_Load(joinpath(dirname(pathof(SimpleDirectMediaLayer)), "..", "assets", "cat.png"))
tex = SDL_CreateTextureFromSurface(renderer, surface)
SDL_FreeSurface(surface)

w_ref, h_ref = Ref{Cint}(0), Ref{Cint}(0)
SDL_QueryTexture(tex, C_NULL, C_NULL, w_ref, h_ref)

try
    w, h = w_ref[], h_ref[]
    x::Float64 = (win_w - w) / 2
    y::Float64 = (win_h - h) / 2
    dest_ref = Ref(SDL_Rect(round(x), round(y), w, h))
    close = false

    speed = 500

    last = SDL_GetPerformanceCounter()
    freq = SDL_GetPerformanceFrequency()

    w_down, a_down, s_down, d_down = false, false, false, false

    while !close
        # Event handling
        event_ref = Ref{SDL_Event}()
        while Bool(SDL_PollEvent(event_ref))
            evt = event_ref[]

            if evt.type == SDL_QUIT
                close = true

            elseif evt.type == SDL_KEYDOWN
                sc = evt.key.keysym.scancode

                if sc == SDL_SCANCODE_W || sc == SDL_SCANCODE_UP
                    w_down = true
                elseif sc == SDL_SCANCODE_A || sc == SDL_SCANCODE_LEFT
                    a_down = true
                elseif sc == SDL_SCANCODE_S || sc == SDL_SCANCODE_DOWN
                    s_down = true
                elseif sc == SDL_SCANCODE_D || sc == SDL_SCANCODE_RIGHT
                    d_down = true
                elseif sc == SDL_SCANCODE_ESCAPE
                    close = true
                end

            elseif evt.type == SDL_KEYUP
                sc = evt.key.keysym.scancode

                if sc == SDL_SCANCODE_W || sc == SDL_SCANCODE_UP
                    w_down = false
                elseif sc == SDL_SCANCODE_A || sc == SDL_SCANCODE_LEFT
                    a_down = false
                elseif sc == SDL_SCANCODE_S || sc == SDL_SCANCODE_DOWN
                    s_down = false
                elseif sc == SDL_SCANCODE_D || sc == SDL_SCANCODE_RIGHT
                    d_down = false
                end
            end
        end

        # Timing
        now = SDL_GetPerformanceCounter()
        dt = (now - last) / freq
        last = now

        # Movement
        vx = (d_down ? 1 : 0) - (a_down ? 1 : 0)
        vy = (s_down ? 1 : 0) - (w_down ? 1 : 0)

        # Normalization
        len = sqrt(vx^2 + vy^2)

        if len > 0
            vx /= len
            vy /= len
        end

        x += vx * speed * dt
        y += vy * speed * dt

        # Edge bounds
        x = clamp(x, 0, win_w - w)
        y = clamp(y, 0, win_h - h)

        # Render
        dest_ref[] = SDL_Rect(
            round(Int, x),
            round(Int, y),
            w,
            h
        )

        SDL_RenderClear(renderer)
        SDL_RenderCopy(renderer, tex, C_NULL, dest_ref)
        SDL_RenderPresent(renderer)
    end
finally
    SDL_DestroyTexture(tex)
    SDL_DestroyRenderer(renderer)
    SDL_DestroyWindow(win)

    SDL_Quit()
end