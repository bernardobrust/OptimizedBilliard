include("src/Data.jl")
include("src/Render.jl")
include("src/Update.jl")

fps_target::Int32 = 120
win_w::Int32 = 800
win_h::Int32 = 450

win, renderer = Render.init_display(win_w, win_h)
data = Data.init_data(fps_target, win_w, win_h, win, renderer)

while Update.update(data)
    Render.render(data)
end

Render.close_display(data)