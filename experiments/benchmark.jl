# =============================================================================
# OptimizedBilliard Benchmark Suite
# Benchmarks SimpleSolution, HomotopySolution, DESolution (0-order), and DESolution (RK4)
# across multiple geometric and dynamic scenarios, and generates native Julia plots.
# =============================================================================

include("../src/Data.jl")
include("../src/SimpleSolution.jl")
include("../src/HomotopySolution.jl")
include("../src/DESolution.jl")
include("plot_utils.jl")

using .Data
using .SimpleSolution
using .HomotopySolution
using .DESolution
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

function benchmark_func(f!, data, sid::Int32; reps::Int=REPETITIONS)
    # Warm up
    for _ in 1:200
        f!(data, sid)
    end
    t0 = time_ns()
    for _ in 1:reps
        f!(data, sid)
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
        "DE (RK4)"        => PlotUtils.COLOR_DE_RK4
    )

    series_names = ["Simple", "DE (Constant)", "DE (RK4)", "Homotopy"]

    # -------------------------------------------------------------------------
    # Experiment 1: Scaling with Number of Balls (4, 8, 16, 32, 64)
    # -------------------------------------------------------------------------
    println("\n[1/3] Benchmarking scaling with number of balls...")
    ball_counts = [4, 8, 16, 32, 64]
    scaling_data = Dict{String, Vector{Float64}}(
        "Simple"        => Float64[],
        "Homotopy"      => Float64[],
        "DE (Constant)" => Float64[],
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
        DESolution.set_predictor_mode!(:zero_order)
        t_de_const = benchmark_func(DESolution.aim_cue!, data, Int32(1))
        push!(scaling_data["DE (Constant)"], t_de_const)

        # DE RK4
        DESolution.set_predictor_mode!(:rk4)
        t_de_rk4 = benchmark_func(DESolution.aim_cue!, data, Int32(1))
        push!(scaling_data["DE (RK4)"], t_de_rk4)

        @printf("  N=%2d balls -> Simple: %6.1f ns | DE (Const): %6.1f ns | DE (RK4): %6.1f ns | Homotopy: %6.1f ns\n",
                n, t_simple, t_de_const, t_de_rk4, t_homo)
    end

    # Plot 1: Ball scaling line plot
    dir = @__DIR__
    out_scaling = joinpath(dir, "results_ball_scaling.svg")
    PlotUtils.plot_line(
        "Execution Time vs. Number of Balls",
        "Number of Balls on Table",
        "Latency (ns / frame)",
        ball_counts,
        scaling_data,
        colors,
        out_scaling
    )

    # -------------------------------------------------------------------------
    # Experiment 2: Scenario Comparisons (Clear LOS, Blocked LOS, Collisions)
    # -------------------------------------------------------------------------
    println("\n[2/3] Benchmarking diverse operational scenarios...")
    categories = ["Clear LOS", "Blocked LOS", "Dense (32 Balls)", "Collision Bounce"]
    scenario_data = Dict{String, Vector{Float64}}(
        "Simple"        => zeros(Float64, length(categories)),
        "DE (Constant)" => zeros(Float64, length(categories)),
        "DE (RK4)"      => zeros(Float64, length(categories)),
        "Homotopy"      => zeros(Float64, length(categories))
    )

    # Scenario 1: Clear LOS (16 balls, unobstructed)
    d_clear = create_data(Int32(16); blocked=false)
    scenario_data["Simple"][1] = benchmark_func(SimpleSolution.aim_cue!, d_clear, Int32(1))
    DESolution.set_predictor_mode!(:zero_order)
    scenario_data["DE (Constant)"][1] = benchmark_func(DESolution.aim_cue!, d_clear, Int32(1))
    DESolution.set_predictor_mode!(:rk4)
    scenario_data["DE (RK4)"][1] = benchmark_func(DESolution.aim_cue!, d_clear, Int32(1))
    scenario_data["Homotopy"][1] = benchmark_func(HomotopySolution.aim_cue!, d_clear, Int32(1))

    # Scenario 2: Blocked LOS (16 balls, single direct blocker)
    d_blocked = create_data(Int32(16); blocked=true)
    scenario_data["Simple"][2] = benchmark_func(SimpleSolution.aim_cue!, d_blocked, Int32(1))
    DESolution.set_predictor_mode!(:zero_order)
    scenario_data["DE (Constant)"][2] = benchmark_func(DESolution.aim_cue!, d_blocked, Int32(1))
    DESolution.set_predictor_mode!(:rk4)
    scenario_data["DE (RK4)"][2] = benchmark_func(DESolution.aim_cue!, d_blocked, Int32(1))
    scenario_data["Homotopy"][2] = benchmark_func(HomotopySolution.aim_cue!, d_blocked, Int32(1))

    # Scenario 3: Dense Field (32 balls)
    d_dense = create_data(Int32(32); blocked=true)
    scenario_data["Simple"][3] = benchmark_func(SimpleSolution.aim_cue!, d_dense, Int32(1))
    DESolution.set_predictor_mode!(:zero_order)
    scenario_data["DE (Constant)"][3] = benchmark_func(DESolution.aim_cue!, d_dense, Int32(1))
    DESolution.set_predictor_mode!(:rk4)
    scenario_data["DE (RK4)"][3] = benchmark_func(DESolution.aim_cue!, d_dense, Int32(1))
    scenario_data["Homotopy"][3] = benchmark_func(HomotopySolution.aim_cue!, d_dense, Int32(1))

    # Scenario 4: Collision Perturbation (Velocity jump every frame)
    d_bounce = create_data(Int32(16); blocked=true)
    function bench_collision(f!, d, sid)
        for _ in 1:200; f!(d, sid); end
        t0 = time_ns()
        for i in 1:REPETITIONS
            # Invert velocity to force Newton collision detector fallback
            if i % 2 == 0
                d.ball_vx[2] = -d.ball_vx[2]
                d.ball_vy[2] = -d.ball_vy[2]
            end
            f!(d, sid)
        end
        return (time_ns() - t0) / REPETITIONS
    end
    scenario_data["Simple"][4] = bench_collision(SimpleSolution.aim_cue!, d_bounce, Int32(1))
    DESolution.set_predictor_mode!(:zero_order)
    scenario_data["DE (Constant)"][4] = bench_collision(DESolution.aim_cue!, d_bounce, Int32(1))
    DESolution.set_predictor_mode!(:rk4)
    scenario_data["DE (RK4)"][4] = bench_collision(DESolution.aim_cue!, d_bounce, Int32(1))
    scenario_data["Homotopy"][4] = bench_collision(HomotopySolution.aim_cue!, d_bounce, Int32(1))

    for (c_idx, cat) in enumerate(categories)
        @printf("  Scenario '%-18s' -> Simple: %6.1f ns | DE (Const): %6.1f ns | DE (RK4): %6.1f ns | Homotopy: %6.1f ns\n",
                cat, scenario_data["Simple"][c_idx], scenario_data["DE (Constant)"][c_idx], scenario_data["DE (RK4)"][c_idx], scenario_data["Homotopy"][c_idx])
    end

    # Plot 2: Scenario comparison grouped bar chart
    out_scenarios = joinpath(dir, "results_scenarios.svg")
    PlotUtils.plot_grouped_bar(
        "Latency Across Operational Scenarios",
        "Latency (ns / frame)",
        categories,
        series_names,
        scenario_data,
        colors,
        out_scenarios
    )

    # -------------------------------------------------------------------------
    # Experiment 3: Summary & Max FPS Capability (Standard 16-Ball Table)
    # -------------------------------------------------------------------------
    println("\n[3/3] Generating summary & FPS comparison...")
    d_std = create_data(Int32(16); blocked=true)
    t_simp = benchmark_func(SimpleSolution.aim_cue!, d_std, Int32(1))
    DESolution.set_predictor_mode!(:zero_order)
    t_dec  = benchmark_func(DESolution.aim_cue!, d_std, Int32(1))
    DESolution.set_predictor_mode!(:rk4)
    t_derk = benchmark_func(DESolution.aim_cue!, d_std, Int32(1))
    t_hom  = benchmark_func(HomotopySolution.aim_cue!, d_std, Int32(1))

    summary_cats = ["Standard 16 Balls"]
    summary_data = Dict{String, Vector{Float64}}(
        "Simple"        => [t_simp],
        "DE (Constant)" => [t_dec],
        "DE (RK4)"      => [t_derk],
        "Homotopy"      => [t_hom]
    )

    out_summary = joinpath(dir, "results_summary.svg")
    PlotUtils.plot_grouped_bar(
        "Standard Billiard Setup Performance (16 Balls)",
        "Latency (ns / frame)",
        summary_cats,
        series_names,
        summary_data,
        colors,
        out_summary
    )

    println("\n-----------------------------------------------------------------")
    println("SUMMARY TABLE (Standard 16-Ball Match):")
    println("-----------------------------------------------------------------")
    @printf("| Solution         | Time / Frame (ns) | Max Theoretical FPS | Relative Speed |\n")
    @printf("|:-----------------|------------------:|--------------------:|---------------:|\n")
    @printf("| Simple           | %17.1f | %19.1f | %13.2fx |\n", t_simp, 1e9 / t_simp, t_hom / t_simp)
    @printf("| DE (Constant)    | %17.1f | %19.1f | %13.2fx |\n", t_dec,  1e9 / t_dec,  t_hom / t_dec)
    @printf("| DE (RK4)         | %17.1f | %19.1f | %13.2fx |\n", t_derk, 1e9 / t_derk, t_hom / t_derk)
    @printf("| Homotopy         | %17.1f | %19.1f | %13.2fx |\n", t_hom,  1e9 / t_hom,  1.0)
    println("-----------------------------------------------------------------")
    println("All plots successfully generated in $(dir)!")
end

run_all_benchmarks()
