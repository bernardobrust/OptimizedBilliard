module Input

using SimpleDirectMediaLayer
using SimpleDirectMediaLayer.LibSDL2

# @TODO: add mouse
const keys = Dict{SDL_Scancode,Bool}()

function handle_input(pressed::Bool, scancode::SDL_Scancode)
    keys[scancode] = pressed
end

function is_down(scancode::SDL_Scancode)::Bool
    get(keys, scancode, false)
end

end