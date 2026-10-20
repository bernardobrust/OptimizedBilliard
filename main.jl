# =============================================================================
# Optimized Billiard - Application Entry Point
# Initializes SDL2 display, font, and game state (DataPlex), then runs the
# real-time update-render loop until the user exits.
# =============================================================================

include("src/Data.jl")
include("src/Render.jl")
include("src/Update.jl")

# Target simulation rate (Hz) and initial window dimensions (pixels)
fps_target::Int32 = 120
win_w::Int32 = 1200
win_h::Int32 = 800

# Initialize SDL2 window, hardware-accelerated renderer, and HUD font
win, renderer = Render.init_display(win_w, win_h)
font = Render.load_font(Int32(16))

# Allocate and populate the main game state structure (balls, cue, timers, HUD)
data = Data.init_data(fps_target, win_w, win_h, win, renderer, font)

# Main interactive loop: process input, advance physics & aiming, and render frame
while Update.update(data)
    Render.render(data)
end

# Release SDL2 window, renderer, textures, and font resources
Render.close_display(data)

