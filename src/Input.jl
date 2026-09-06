module Input

using SimpleDirectMediaLayer
using SimpleDirectMediaLayer.LibSDL2

const key_states = zeros(Bool, 512)

const mouse_pos = Float32[0.0f0, 0.0f0]
const mouse_buttons = Bool[false, false] # [is_down, was_pressed]

function end_frame()
    mouse_buttons[2] = false
end

function handle_key(pressed::Bool, scancode)
    idx = Int(scancode) + 1
    if idx >= 1 && idx <= length(key_states)
        @inbounds key_states[idx] = pressed
    end
end

function is_down(scancode)::Bool
    idx = Int(scancode) + 1
    if idx >= 1 && idx <= length(key_states)
        @inbounds return key_states[idx]
    end
    false
end

function handle_mouse_button(pressed::Bool, button, x, y)
    mouse_pos[1] = Float32(x)
    mouse_pos[2] = Float32(y)
    if button == SDL_BUTTON_LEFT
        if pressed && !mouse_buttons[1]
            mouse_buttons[2] = true
        end
        mouse_buttons[1] = pressed
    end
end

function handle_mouse_motion(x, y)
    mouse_pos[1] = Float32(x)
    mouse_pos[2] = Float32(y)
end

mouse_x()::Float32 = mouse_pos[1]
mouse_y()::Float32 = mouse_pos[2]
mouse_left_down()::Bool = mouse_buttons[1]
mouse_left_pressed()::Bool = mouse_buttons[2]

end