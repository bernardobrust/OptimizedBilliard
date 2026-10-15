# Optimized Billiard
## The Idea
This project is an optimized implementation of a billiard-like scenario. Multiple "balls" (circles) move around a table and are tracked by a "cue" (a line).

Our goal is simple: given a position on a ball, make the cue track that position and draw a line from the cue to the first collision. The first collision may be with another ball blocking the tracked ball.

*We explore three different solutions:*

### **Simple**
We iterate over every ball at every frame and calculate the intersection between the cue and each ball. The line stops at the closest intersection.

This approach is straightforward, but its computational cost increases with the number of balls and frames.

### **Differential Equations**
We find the point to track in the first frame using the simple solution. We then build a nonlinear system of equations representing the ball and the cue line.

From this system, we construct a Jacobian matrix and use numerical integration together with Newton's Method to correct the error. This allows us to predict where the intersection point will move instead of recomputing it from scratch every frame.

### **Homotopy Continuation**
The previous approach may require solving a nonlinear system repeatedly. Although our particular problem is relatively simple (a line-circle intersection), we wanted to explore a more general technique that can also be applied to systems with many solutions.

With Homotopy Continuation, we start by solving a simpler version of the problem and continuously transform it into the actual system we want to solve.

Resources on Homotopy Continuation can be found [here](https://homotopycontinuation.github.io/). An implementation of the line-circle problem can also be found [here](https://github.com/rfabbri/minus).

## Context
This project was built by three students:
- [Bernardo Brust](https://github.com/bernardobrust)
- [Felipe Feliciano](https://github.com/FelipePF)
- [Gabriel Andrade](https://github.com/bielhxx)

It was developed for the elective course **"Special Topics in Programming Languages"**, which had three main goals:

- Learn a new programming language. We chose **Julia**, following our professor's recommendation.
- Solve an interesting computational geometry problem in the process.
- Learn new mathematical tools for solving real-world problems.

None of us had much experience with Julia before starting this project, so the code may not always follow Julia best practices. Please keep that in mind!

The original implementation was first sketched in Python. This repository contains the final implementation in Julia.

## Running
### Requirements
- The [Julia Language](https://julialang.org/)
- [SDL2](https://www.libsdl.org/) for [Julia](https://github.com/JuliaMultimedia/SimpleDirectMediaLayer.jl)

### Running
Julia uses a JIT compiler, so you can just call the compiler against main:
```bash
julia main.jl
```

On Wayland you may be required to switch to X11 and use `XWayland`:
```bash
SDL_VIDEODRIVER=x11 julia main.jl
```

To run with optimizations enabled:
```bash
julia -O3 -g0 main.jl
```

### Controls
- **Left Mouse Click**: Aim cue at clicked location or ball.
- **S Key**: Cycle through aiming calculation methods:
  - `Simple`: Closed-form geometric quadratic line-circle solver.
  - `Homotopy`: Polynomial homotopy path tracking.
  - `DE (Constant)`: Davidenko differential continuation with frame-to-frame temporal tracking and Newton displacement collision detection.
  - `DE (Euler)`: First-order Euler prediction followed by Newton correction.
  - `DE (RK4)`: 4th-order Runge-Kutta numerical integration of the Davidenko vector field.
- **Escape**: Exit application.

## Experiments and Results

Run `julia experiments/benchmark.jl` to regenerate the benchmark SVGs, or `julia experiments/tracked_point_trajectories.jl` to regenerate the tracking data and its separate solution plots. The benchmark advances a deterministic 16-ball scene, clears each DE solver's continuation history before each run, and performs the game's pre-physics and post-physics aim updates. It includes ball integration, wall reflections, and pair collisions; SDL rendering and event processing are excluded. Results are therefore simulated-update timings, not full-game FPS.

### Benchmark Results

The benchmark confirms that Simple is the cheapest method for this small two-dimensional line-circle problem. Its ray checks use direct geometric calculations. The DE methods add prediction and correction work, while Homotopy tracks polynomial roots through a continuation path. For this problem, those costs outweigh any savings from temporal tracking.

The latest 16-ball run measured approximately 243 ns/update for Simple, 348 ns for DE 0th, 358 ns for DE Euler, 378 ns for DE RK4, and 653 ns for Homotopy. These figures vary by machine and runtime; the plots show the full scaling and scenario measurements.

![Update latency as the number of balls increases](experiments/results_ball_scaling.svg)

![Update latency across clear, blocked, dense, and moving-line-of-sight scenes](experiments/results_scenarios.svg)

![Summary update latency for the 16-ball scene](experiments/results_summary.svg)

The methods solve the same geometric first-hit problem, so their tracked paths nearly overlap. In the 600-frame experiment, each DE and Homotopy path differs from the Simple reference by at most about 0.003 pixels. The experiment CSV records the hit type for every frame.

At about 1.12 seconds, the first-hit point jumps because the moving blocker leaves the cue ray. The closest intersection then changes from that blocker to the selected target ball. This geometric change occurs in Simple and all three DE variants as well as Homotopy; it is not a numerical blow-up. The plots break the lines at blocker-to-target switches so the transition does not look like a continuous trajectory.

#### Tracked first-hit point over time

Each plot shows one method against the Simple reference. The x and y coordinates are shown in separate panels; line breaks mark a change between hitting the blocker and hitting the target.

![DE 0th tracked first-hit point](experiments/tracked_point_de_0th.svg)

![DE Euler tracked first-hit point](experiments/tracked_point_de_euler.svg)

![DE RK4 tracked first-hit point](experiments/tracked_point_de_rk4.svg)

![Homotopy tracked first-hit point](experiments/tracked_point_homotopy.svg)

Simple is expected to do well here: it evaluates inexpensive closed-form line-circle intersections in only two dimensions. Higher-order predictors and continuation methods cannot remove the work of considering the candidate balls, and they add their own numerical work. In higher dimensions, with more complicated geometric constraints, or with polynomial systems that lack a cheap direct solution, methods such as predictor-corrector continuation or Homotopy can become more useful and may outperform a direct approach. That depends on the problem and should be measured rather than assumed.

The benchmark renders SVG directly. If `rsvg-convert` is installed, it also creates PNG copies.
