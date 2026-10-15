# Deterministic game-sized time series for the DE predictors and Homotopy.
include("../src/Data.jl")
include("../src/SimpleSolution.jl")
include("../src/SimpleSolution.jl")
include("../src/HomotopySolution.jl")
include("../src/DESolution.jl")
include("../src/DEConstantSolution.jl")
include("../src/DEEulerSolution.jl")
include("../src/DERK4Solution.jl")
using .Data, .SimpleSolution, .DESolution, .HomotopySolution
using .DEConstantSolution, .DEEulerSolution, .DERK4Solution
using Printf

const STEP = 1.0 / 120.0
const FRAMES = 600

function make_scene()
    n = Int32(16)
    ball_x = zeros(Float32, n)
    ball_y = zeros(Float32, n)
    ball_vx = zeros(Float32, n)
    ball_vy = zeros(Float32, n)
    ball_x[1:3] .= Float32[760, 415, 800]
    ball_y[1:3] .= Float32[520, 365, 200]
    ball_vx[1:3] .= Float32[-35, 12, -20]
    ball_vy[1:3] .= Float32[22, -8, 15]
    # Keep the remaining game-count balls below the tracking ray, with no motion.
    for i in 4:n
        ball_x[i] = 50f0 + 80f0 * (i - 4)
        ball_y[i] = 750f0
    end
    Data.DataPlex(UInt64(0), UInt64(1), STEP, 1200f0, 800f0, n, 18f0,
        ball_x, ball_y, ball_vx, ball_vy,
        100f0, 200f0, 16f0, 220f0, 0f0, false, 0f0, 0f0, Int32(1),
        24f0, -4f0, 0f0, 0f0, 0f0, 0f0, 0f0, 0f0, true,
        Ptr{Data.SDL_Window}(C_NULL), Ptr{Data.SDL_Renderer}(C_NULL),
        Ptr{Data.SDL_Texture}(C_NULL), false, Int32(0), :simple,
        Ptr{Data.TTF_Font}(C_NULL), 0.0, 0.0, zeros(100), 1, 1, 100)
end

function advance!(d)
    dt = Float32(STEP)
    for i in 1:d.num_balls
        x = d.ball_x[i] + d.ball_vx[i] * dt
        y = d.ball_y[i] + d.ball_vy[i] * dt
        if x < d.ball_radius
            d.ball_x[i] = d.ball_radius
            d.ball_vx[i] = -d.ball_vx[i]
        elseif x > d.bound_x - d.ball_radius
            d.ball_x[i] = d.bound_x - d.ball_radius
            d.ball_vx[i] = -d.ball_vx[i]
        else
            d.ball_x[i] = x
        end
        if y < d.ball_radius
            d.ball_y[i] = d.ball_radius
            d.ball_vy[i] = -d.ball_vy[i]
        elseif y > d.bound_y - d.ball_radius
            d.ball_y[i] = d.bound_y - d.ball_radius
            d.ball_vy[i] = -d.ball_vy[i]
        else
            d.ball_y[i] = y
        end
    end
end

function run_series(name, aim!, mode=nothing)
    d = make_scene()
    mode === nothing || DESolution.set_predictor_mode!(mode)
    DESolution.PREV_TRACKED_BALL[] = 0
    DESolution.PREV_DELTA_X[] = 0.0
    DESolution.PREV_DELTA_Y[] = 0.0
    points = Vector{NTuple{2,Float64}}(undef, FRAMES)
    for i in 1:FRAMES
        aim!(d, Int32(1))
        points[i] = (Float64(d.collision_x), Float64(d.collision_y))
        advance!(d)
    end
    return name => points
end

series = [
    run_series("DE 0th", DEConstantSolution.aim_cue!),
    run_series("DE Euler", DEEulerSolution.aim_cue!),
    run_series("DE RK4", DERK4Solution.aim_cue!),
    run_series("Homotopy", HomotopySolution.aim_cue!)
]
reference = run_series("Simple reference", SimpleSolution.aim_cue!)

csv_path = joinpath(@__DIR__, "tracked_point_trajectories.csv")
open(csv_path, "w") do io
    println(io, "frame,time_s,solution,x,y")
    for (name, points) in vcat([reference], series), i in eachindex(points)
        @printf(io, "%d,%.6f,%s,%.6f,%.6f\n", i, (i - 1) * STEP, name, points[i]...)
    end
end

colors = Dict("DE 0th" => "#1976D2", "DE Euler" => "#388E3C", "DE RK4" => "#E65100", "Homotopy" => "#C2185B")
const WIDTH, HEIGHT, LEFT, TOP, PLOT_W, PLOT_H = 1000, 700, 85, 55, 850, 255
for (name, points) in series
    svg_path = joinpath(@__DIR__, "tracked_point_$(replace(lowercase(name), ' ' => '_')).svg")
    both = (points, reference[2])
    open(svg_path, "w") do io
        println(io, "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"$WIDTH\" height=\"$HEIGHT\" viewBox=\"0 0 $WIDTH $HEIGHT\">")
        println(io, "<rect width=\"100%\" height=\"100%\" fill=\"white\"/>")
        for (panel, coord, title) in ((0, 1, "LOS first-hit x over time"), (1, 2, "LOS first-hit y over time"))
            vals = [pt[coord] for pts in both for pt in pts]
            lo, hi = extrema(vals)
            pad = max((hi - lo) * 0.05, 1.0)
            lo -= pad; hi += pad
            y0 = TOP + panel * 325
            xpx(i) = LEFT + (i - 1) / (FRAMES - 1) * PLOT_W
            ypx(v) = y0 + PLOT_H - (v - lo) / (hi - lo) * PLOT_H
            println(io, "<text x=\"500\" y=\"$(y0 - 18)\" text-anchor=\"middle\" font-size=\"18\">$name: $title</text>")
            println(io, "<path d=\"M $LEFT $y0 V $(y0+PLOT_H) H $(LEFT+PLOT_W)\" fill=\"none\" stroke=\"#555\"/>")
            for second in 0:5
                frame = round(Int, 1 + second / ((FRAMES - 1) * STEP) * (FRAMES - 1))
                xp = xpx(frame)
                println(io, "<path d=\"M $xp $(y0+PLOT_H) v 5\" stroke=\"#555\"/>")
                println(io, "<text x=\"$xp\" y=\"$(y0+PLOT_H+18)\" text-anchor=\"middle\" font-size=\"11\">$(second)s</text>")
            end
            for (pts, color, label) in ((reference[2], "#777777", "Simple reference"), (points, colors[name], name))
                path = join((string(i == 1 ? "M" : "L", " ", round(xpx(i), digits=1), " ", round(ypx(pt[coord]), digits=1)) for (i, pt) in enumerate(pts)), " ")
                println(io, "<path d=\"$path\" fill=\"none\" stroke=\"$color\" stroke-width=\"2\"/>")
                println(io, "<text x=\"$(LEFT + (label == "Simple reference" ? 0 : 180))\" y=\"$(y0+PLOT_H+24)\" fill=\"$color\" font-size=\"13\">$label</text>")
            end
            println(io, "<text x=\"500\" y=\"$(y0+PLOT_H+43)\" text-anchor=\"middle\" font-size=\"12\">Time (s), 1/120-second frames</text>")
        end
        println(io, "</svg>")
    end
    println("Wrote $svg_path")
end
println("Wrote $csv_path")
