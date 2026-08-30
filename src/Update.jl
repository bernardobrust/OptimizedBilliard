module Update

include("Input.jl")

using SimpleDirectMediaLayer
using SimpleDirectMediaLayer.LibSDL2

function update(data)::Bool
    # Event handling
    event_ref = Ref{SDL_Event}()
    while Bool(SDL_PollEvent(event_ref))
        evt = event_ref[]

        if evt.type == SDL_QUIT
            return false

        elseif evt.type == SDL_KEYDOWN
            sc = evt.key.keysym.scancode
            Input.handle_input(true, sc)

        elseif evt.type == SDL_KEYUP
            sc = evt.key.keysym.scancode
            Input.handle_input(false, sc)
        end
    end

    Input.is_down(SDL_SCANCODE_ESCAPE) && (return false)

    # Timing
    now = SDL_GetPerformanceCounter()
    dt = (now - data.last) / data.freq
    data.last = now

    # Movement
    # @TODO: set the cue position to be the mouse position when sliding with left click
    vx = (Input.is_down(SDL_SCANCODE_D) ? 1 : 0) - (Input.is_down(SDL_SCANCODE_A) ? 1 : 0)
    vy = (Input.is_down(SDL_SCANCODE_S) ? 1 : 0) - (Input.is_down(SDL_SCANCODE_W) ? 1 : 0)

    # @TODO: newtonian mechanics

    # @TODO: track selected ball, allow the user to select the tracking method

    # Normalization
    len = sqrt(vx^2 + vy^2)
    if len > 0
        vx /= len
        vy /= len
    end

    data.x += vx * data.speed * dt
    data.y += vy * data.speed * dt

    # Edge bounds
    data.x = clamp(data.x, 0, data.bound_x - data.w)
    data.y = clamp(data.y, 0, data.bound_y - data.h)

    # Keep updating
    true
end

end