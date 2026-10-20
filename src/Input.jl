module Input

# =============================================================================
# Input Module
# Tracks keyboard scancodes and left mouse button / cursor state across frames.
# Distinguishes between continuous hold (`is_down`) and single-frame rising-edge
# click events (`was_pressed`).
# =============================================================================

using SimpleDirectMediaLayer
using SimpleDirectMediaLayer.LibSDL2

# Direct-indexed table of SDL scancode states (true = key currently held down)
const key_states = zeros(Bool, 512)

# Current cursor coordinates [x, y] in window pixels
const mouse_pos = Float32[0.0f0, 0.0f0]

# Left mouse button state: [1] = is_down (continuous), [2] = was_pressed (this frame only)
const mouse_buttons = Bool[false, false] # [is_down, was_pressed]

# Resets single-frame edge-triggered press flags at the end of each update tick
@inline function end_frame()
    mouse_buttons[2] = false
end

# Records key press/release events indexed by SDL scancode (+1 for 1-based Julia indexing)
@inline function handle_key(pressed::Bool, scancode)
    idx = Int(scancode) + 1
    if idx >= 1 && idx <= length(key_states)
        @inbounds key_states[idx] = pressed
    end
end

# Queries whether the given SDL scancode is currently held down
@inline function is_down(scancode)::Bool
    idx = Int(scancode) + 1
    if idx >= 1 && idx <= length(key_states)
        @inbounds return key_states[idx]
    end
    false
end

# Updates cursor position and left mouse button state on SDL mouse button events
@inline function handle_mouse_button(pressed::Bool, button, x, y)
    mouse_pos[1] = Float32(x)
    mouse_pos[2] = Float32(y)
    if button == SDL_BUTTON_LEFT
        # Trigger single-frame `was_pressed` only on the rising edge (!is_down -> pressed)
        if pressed && !mouse_buttons[1]
            mouse_buttons[2] = true
        end
        mouse_buttons[1] = pressed
    end
end

# Updates cursor coordinates on SDL mouse motion events
@inline function handle_mouse_motion(x, y)
    mouse_pos[1] = Float32(x)
    mouse_pos[2] = Float32(y)
end

mouse_x()::Float32 = mouse_pos[1]
mouse_y()::Float32 = mouse_pos[2]
mouse_left_down()::Bool = mouse_buttons[1]
mouse_left_pressed()::Bool = mouse_buttons[2]

end

