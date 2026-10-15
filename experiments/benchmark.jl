# =============================================================================
# OptimizedBilliard Benchmark Suite
# Measures simulated updates using the game's ball integration, wall reflection,
# pair collisions, and aim solver. Rendering and event handling are excluded.
# =============================================================================

include("../src/Data.jl")
include("../src/SimpleSolution.jl")
include("../src/HomotopySolution.jl")
include("../src/DESolution.jl")
include("../src/DEConstantSolution.jl")
include("../src/DEEulerSolution.jl")
include("../src/DERK4Solution.jl")
include("plot_utils.jl")

using .Data
using .SimpleSolution
using .HomotopySolution
using .DESolution
using .DEConstantSolution, .DEEulerSolution, .DERK4Solution
using .PlotUtils
using Printf
using Random

const REPETITIONS = 15_000

function create_data(num_balls::Int32; blocked::Bool=true)
    ball_radius = 18.0f0
    ball_x = zeros(Float32, num_balls)
    ball_y = zeros(Float32, num_balls)
    ball_vx = zeros(Float32, num_balls)
    ball_vy = zeros(Float32, num_balls)

    # Ball 1 is target ball
    ball_x[1] = 700.0f0
    ball_y[1] = 500.0f0
    ball_vx[1] = -40.0f0
    ball_vy[1] = 30.0f0

    if blocked && num_balls >= 2
        # Ball 2 directly blocks LOS between cue (100, 200) and target (700, 500)
        ball_x[2] = 400.0f0
        ball_y[2] = 350.0f0
        ball_vx[2] = 60.0f0
        ball_vy[2] = -45.0f0
    elseif num_balls >= 2
        # Ball 2 out of way
        ball_x[2] = 100.0f0
        ball_y[2] = 700.0f0
    end

    # Place other balls
    rng = MersenneTwister(42)
    for i in 3:num_balls
        ball_x[i] = 100.0f0 + rand(rng, Float32) * 1000.0f0
        ball_y[i] = 100.0f0 + rand(rng, Float32) * 600.0f0
        ball_vx[i] = -80.0f0 + rand(rng, Float32) * 160.0f0
        ball_vy[i] = -80.0f0 + rand(rng, Float32) * 160.0f0
    end

    Data.DataPlex(
        UInt64(0), UInt64(1000), 1.0/120.0,
        1200.0f0, 800.0f0,
        num_balls, ball_radius,
        ball_x, ball_y, ball_vx, ball_vy,
        100.0f0, 200.0f0, 16.0f0, 220.0f0, 0.0f0,
        false, 0.0f0, 0.0f0, Int32(1), 0.0f0, 0.0f0,
        0.0f0, 0.0f0, 0.0f0, 0.0f0, 0.0f0, 0.0f0, true,
        Ptr{Data.SDL_Window}(C_NULL), Ptr{Data.SDL_Renderer}(C_NULL), Ptr{Data.SDL_Texture}(C_NULL),
        false, Int32(0), :simple, Ptr{Data.TTF_Font}(C_NULL),
        0.0, 0.0, zeros(Float64, 100), 1, 1, 100
    )
end

function reset_de_state!()
    DESolution.PREV_TRACKED_BALL[] = 0
    DESolution.PREV_DELTA_X[] = 0.0
    DESolution.PREV_DELTA_Y[] = 0.0
end

function advance_game_physics!(data)
    dt = Float32(data.dt_target)
    rad = data.ball_radius
    max_x, max_y = data.bound_x - rad, data.bound_y - rad
    @inbounds for i in 1:data.num_balls
        x = data.ball_x[i] + data.ball_vx[i] * dt
        y = data.ball_y[i] + data.ball_vy[i] * dt
        vx, vy = data.ball_vx[i], data.ball_vy[i]
        if x < rad
            x = rad; vx = -vx
        elseif x > max_x
            x = max_x; vx = -vx
        end
        if y < rad
            y = rad; vy = -vy
        elseif y > max_y
            y = max_y; vy = -vy
        end
        data.ball_x[i], data.ball_y[i] = x, y
        data.ball_vx[i], data.ball_vy[i] = vx, vy
    end

    min_dist = 2.0f0 * rad
    min_dist_sq = min_dist * min_dist
    @inbounds for i in 1:(data.num_balls-1)
        x1, y1 = data.ball_x[i], data.ball_y[i]
        vx1, vy1 = data.ball_vx[i], data.ball_vy[i]
        for j in (i+1):data.num_balls
            x2, y2 = data.ball_x[j], data.ball_y[j]
            dx, dy = x2 - x1, y2 - y1
            dist_sq = dx * dx + dy * dy
            if dist_sq < min_dist_sq
                dist = sqrt(dist_sq)
                nx, ny = dist == 0.0f0 ? (1.0f0, 0.0f0) : (dx / dist, dy / dist)
                overlap = 0.5f0 * (min_dist - dist)
                x1 -= nx * overlap; y1 -= ny * overlap
                data.ball_x[j] = x2 + nx * overlap
                data.ball_y[j] = y2 + ny * overlap
                vx2, vy2 = data.ball_vx[j], data.ball_vy[j]
                rel_speed = (vx1 - vx2) * nx + (vy1 - vy2) * ny
                if rel_speed > 0.0f0
                    vx1 -= rel_speed * nx; vy1 -= rel_speed * ny
                    data.ball_vx[j] = vx2 + rel_speed * nx
                    data.ball_vy[j] = vy2 + rel_speed * ny
                end
            end
        end
        data.ball_x[i], data.ball_y[i] = x1, y1
        data.ball_vx[i], data.ball_vy[i] = vx1, vy1
    end
end

function benchmark_func(f!, source_data, sid::Int32; reps::Int=REPETITIONS)
    data = deepcopy(source_data)
    initial_x, initial_y = copy(data.ball_x), copy(data.ball_y)
    initial_vx, initial_vy = copy(data.ball_vx), copy(data.ball_vy)
    # The game aims before physics and realigns once more after physics.
    step!() = (f!(data, sid); advance_game_physics!(data); f!(data, sid))
    reset!() = begin
        copyto!(data.ball_x, initial_x); copyto!(data.ball_y, initial_y)
        copyto!(data.ball_vx, initial_vx); copyto!(data.ball_vy, initial_vy)
        data.collision_x = 0.0f0; data.collision_y = 0.0f0
        reset_de_state!()
    end
    reset!()
    for _ in 1:200
        step!()
    end
    reset!()
    t0 = time_ns()
    for _ in 1:reps
        step!()
    end
    t_total = time_ns() - t0
    return t_total / reps
end

function run_all_benchmarks()
    println("=================================================================")
    println("          OPTIMIZED BILLIARD - BENCHMARK SUITE                   ")
    println("=================================================================")

    colors = Dict(
        "Simple"          => PlotUtils.COLOR_SIMPLE,
        "Homotopy"        => PlotUtils.COLOR_HOMOTOPY,
        "DE (Constant)"   => PlotUtils.COLOR_DE_0TH,
        "DE (Euler)"      => PlotUtils.COLOR_DE_EULER,
        "DE (RK4)"        => PlotUtils.COLOR_DE_RK4
    )

    series_names = ["Simple", "DE (Constant)", "DE (Euler)", "DE (RK4)", "Homotopy"]

    # -------------------------------------------------------------------------
    # Experiment 1: Scaling with Number of Balls (4, 8, 16, 32, 64)
    # -------------------------------------------------------------------------
    println("\n[1/3] Benchmarking scaling with number of balls...")
    ball_counts = [4, 8, 16, 32, 64]
    scaling_data = Dict{String, Vector{Float64}}(
        "Simple"        => Float64[],
        "Homotopy"      => Float64[],
        "DE (Constant)" => Float64[],
        "DE (Euler)"    => Float64[],
        "DE (RK4)"      => Float64[]
    )

    for n in ball_counts
        data = create_data(Int32(n); blocked=true)

        # Simple
        t_simple = benchmark_func(SimpleSolution.aim_cue!, data, Int32(1))
        push!(scaling_data["Simple"], t_simple)

        # Homotopy
        t_homo = benchmark_func(HomotopySolution.aim_cue!, data, Int32(1))
        push!(scaling_data["Homotopy"], t_homo)

        # DE Constant
        t_de_const = benchmark_func(DEConstantSolution.aim_cue!, data, Int32(1))
        push!(scaling_data["DE (Constant)"], t_de_const)

        t_de_euler = benchmark_func(DEEulerSolution.aim_cue!, data, Int32(1))
        push!(scaling_data["DE (Euler)"], t_de_euler)

        # DE RK4
        t_de_rk4 = benchmark_func(DERK4Solution.aim_cue!, data, Int32(1))
        push!(scaling_data["DE (RK4)"], t_de_rk4)

        @printf("  N=%2d -> Simple: %6.1f | 0th: %6.1f | Euler: %6.1f | RK4: %6.1f | Homotopy: %6.1f ns\n",
                n, t_simple, t_de_const, t_de_euler, t_de_rk4, t_homo)
    end

    # Plot 1: Ball scaling line plot
    dir = @__DIR__
    out_scaling = joinpath(dir, "results_ball_scaling.svg")
    PlotUtils.plot_line(
        "Execution Time vs. Number of Balls",
        "Number of Balls on Table",
        "Latency (ns / simulated update)",
        ball_counts,
        scaling_data,
        colors,
        out_scaling
    )

    # -------------------------------------------------------------------------
    # Experiment 2: Scenario Comparisons (Clear LOS, Blocked LOS, Collisions)
    # -------------------------------------------------------------------------
    println("\n[2/3] Benchmarking diverse operational scenarios...")
    categories = ["Clear LOS", "Blocked LOS", "Dense (32 Balls)", "Moving LOS"]
    scenario_data = Dict{String, Vector{Float64}}(
        "Simple"        => zeros(Float64, length(categories)),
        "DE (Constant)" => zeros(Float64, length(categories)),
        "DE (Euler)"    => zeros(Float64, length(categories)),
        "DE (RK4)"      => zeros(Float64, length(categories)),
        "Homotopy"      => zeros(Float64, length(categories))
    )

    # Scenario 1: Clear LOS (16 balls, unobstructed)
    d_clear = create_data(Int32(16); blocked=false)
    scenario_data["Simple"][1] = benchmark_func(SimpleSolution.aim_cue!, d_clear, Int32(1))
    scenario_data["DE (Constant)"][1] = benchmark_func(DEConstantSolution.aim_cue!, d_clear, Int32(1))
    scenario_data["DE (Euler)"][1] = benchmark_func(DEEulerSolution.aim_cue!, d_clear, Int32(1))
    scenario_data["DE (RK4)"][1] = benchmark_func(DERK4Solution.aim_cue!, d_clear, Int32(1))
    scenario_data["Homotopy"][1] = benchmark_func(HomotopySolution.aim_cue!, d_clear, Int32(1))

    # Scenario 2: Blocked LOS (16 balls, single direct blocker)
    d_blocked = create_data(Int32(16); blocked=true)
    scenario_data["Simple"][2] = benchmark_func(SimpleSolution.aim_cue!, d_blocked, Int32(1))
    scenario_data["DE (Constant)"][2] = benchmark_func(DEConstantSolution.aim_cue!, d_blocked, Int32(1))
    scenario_data["DE (Euler)"][2] = benchmark_func(DEEulerSolution.aim_cue!, d_blocked, Int32(1))
    scenario_data["DE (RK4)"][2] = benchmark_func(DERK4Solution.aim_cue!, d_blocked, Int32(1))
    scenario_data["Homotopy"][2] = benchmark_func(HomotopySolution.aim_cue!, d_blocked, Int32(1))

    # Scenario 3: Dense Field (32 balls)
    d_dense = create_data(Int32(32); blocked=true)
    scenario_data["Simple"][3] = benchmark_func(SimpleSolution.aim_cue!, d_dense, Int32(1))
    scenario_data["DE (Constant)"][3] = benchmark_func(DEConstantSolution.aim_cue!, d_dense, Int32(1))
    scenario_data["DE (Euler)"][3] = benchmark_func(DEEulerSolution.aim_cue!, d_dense, Int32(1))
    scenario_data["DE (RK4)"][3] = benchmark_func(DERK4Solution.aim_cue!, d_dense, Int32(1))
    scenario_data["Homotopy"][3] = benchmark_func(HomotopySolution.aim_cue!, d_dense, Int32(1))

    # Scenario 4: Moving LOS under the same per-update physics step
    d_moving = create_data(Int32(16); blocked=true)
    scenario_data["Simple"][4] = benchmark_func(SimpleSolution.aim_cue!, d_moving, Int32(1))
    scenario_data["DE (Constant)"][4] = benchmark_func(DEConstantSolution.aim_cue!, d_moving, Int32(1))
    scenario_data["DE (Euler)"][4] = benchmark_func(DEEulerSolution.aim_cue!, d_moving, Int32(1))
    scenario_data["DE (RK4)"][4] = benchmark_func(DERK4Solution.aim_cue!, d_moving, Int32(1))
    scenario_data["Homotopy"][4] = benchmark_func(HomotopySolution.aim_cue!, d_moving, Int32(1))

    for (c_idx, cat) in enumerate(categories)
        @printf("  Scenario '%-18s' -> Simple: %6.1f | 0th: %6.1f | Euler: %6.1f | RK4: %6.1f | Homotopy: %6.1f ns\n",
                cat, scenario_data["Simple"][c_idx], scenario_data["DE (Constant)"][c_idx], scenario_data["DE (Euler)"][c_idx], scenario_data["DE (RK4)"][c_idx], scenario_data["Homotopy"][c_idx])
    end

    # Plot 2: Scenario comparison grouped bar chart
    out_scenarios = joinpath(dir, "results_scenarios.svg")
    PlotUtils.plot_grouped_bar(
        "Latency Across Operational Scenarios",
        "Latency (ns / simulated update)",
        categories,
        series_names,
        scenario_data,
        colors,
        out_scenarios
    )

    # -------------------------------------------------------------------------
    # Experiment 3: Simulated-update latency summary (Standard 16-Ball Scene)
    # -------------------------------------------------------------------------
    println("\n[3/3] Generating simulated-update latency summary...")
    d_std = create_data(Int32(16); blocked=true)
    t_simp = benchmark_func(SimpleSolution.aim_cue!, d_std, Int32(1))
    t_dec  = benchmark_func(DEConstantSolution.aim_cue!, d_std, Int32(1))
    t_dee  = benchmark_func(DEEulerSolution.aim_cue!, d_std, Int32(1))
    t_derk = benchmark_func(DERK4Solution.aim_cue!, d_std, Int32(1))
    t_hom  = benchmark_func(HomotopySolution.aim_cue!, d_std, Int32(1))

    summary_cats = ["Standard 16 Balls"]
    summary_data = Dict{String, Vector{Float64}}(
        "Simple"        => [t_simp],
        "DE (Constant)" => [t_dec],
        "DE (Euler)"    => [t_dee],
        "DE (RK4)"      => [t_derk],
        "Homotopy"      => [t_hom]
    )

    out_summary = joinpath(dir, "results_summary.svg")
    PlotUtils.plot_grouped_bar(
        "Simulated Update Latency (16-Ball Scene)",
        "Latency (ns / update)",
        summary_cats,
        series_names,
        summary_data,
        colors,
        out_summary
    )

    println("\n-----------------------------------------------------------------")
    println("SIMULATED UPDATE LATENCY (aim + ball physics; rendering excluded):")
    println("-----------------------------------------------------------------")
    @printf("| Solution         | Update (ns) | Relative latency |\n")
    @printf("|:-----------------|------------:|-----------------:|\n")
    @printf("| Simple           | %11.1f | %16.2fx |\n", t_simp, t_simp / t_simp)
    @printf("| DE (Constant)    | %11.1f | %16.2fx |\n", t_dec,  t_dec / t_simp)
    @printf("| DE (Euler)       | %11.1f | %16.2fx |\n", t_dee,  t_dee / t_simp)
    @printf("| DE (RK4)         | %11.1f | %16.2fx |\n", t_derk, t_derk / t_simp)
    @printf("| Homotopy         | %11.1f | %16.2fx |\n", t_hom,  t_hom / t_simp)
    println("-----------------------------------------------------------------")
    println("All plots successfully generated in $(dir)!")
end

run_all_benchmarks()
