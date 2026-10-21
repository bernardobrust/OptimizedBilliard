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
  - `DE (Constant)`: Zero-order continuation with frame-to-frame temporal tracking and Newton displacement collision detection (bypasses differential equation integration).
  - `DE (Euler)`: First-order Euler prediction along the Davidenko differential equation followed by Newton correction.
  - `DE (RK4)`: 4th-order Runge-Kutta numerical integration of the Davidenko vector field followed by Newton correction.
- **Escape**: Exit application.

## Experiments and Results

Run `julia -O3 -g0 experiments/benchmark.jl` to regenerate the benchmark SVGs, or `julia -O3 -g0 experiments/tracked_point_trajectories.jl` to regenerate the tracking data and its separate solution plots. All solver hot loops and physics updates are annotated with `@inline` and `@fastmath` and executed with compiler optimizations `-O3 -g0` over `REPETITIONS = 10,000,000` simulated updates. The benchmark advances a deterministic 16-ball scene, clears each DE solver's continuation history before each run, and performs the game's physics and aim updates (ball integration, wall reflections, and pair collisions; SDL rendering and event processing are excluded). Results are therefore simulated-update timings, not full-game FPS.

### Benchmark Results (`julia -O3 -g0`, 10,000,000 Runs)

To capture both average throughput and frame-time stability (tail latency and jitter), the benchmark measures **Mean**, **Median**, **Standard Deviation (`Std Dev`)**, **Best 10%** (mean of the 10% fastest updates, corresponding to unobstructed or fast-path frames), and **Worst 10%** (mean of the 10% slowest updates, corresponding to multi-ball collisions and central-loop continuation frames):

```text
------------------------------------------------------------------------------------------------------
SIMULATED UPDATE LATENCY DISTRIBUTION (16-Ball Scene, 10M Runs; rendering excluded):
------------------------------------------------------------------------------------------------------
| Solution         | Mean (ns) | Median (ns) | Std Dev (ns) | Best 10% (ns) | Worst 10% (ns) | Relative |
|:-----------------|----------:|------------:|-------------:|--------------:|---------------:|---------:|
| Simple           |     146.6 |       150.2 |         37.5 |         120.7 |          177.4 |    1.00x |
| DE (Constant)    |     176.9 |       168.2 |         41.2 |         150.7 |          222.5 |    1.21x |
| DE (Euler)       |     183.3 |       172.8 |         38.4 |         156.2 |          231.8 |    1.25x |
| DE (RK4)         |     192.0 |       195.5 |         48.4 |         156.9 |          255.5 |    1.31x |
| Homotopy         |     391.1 |       216.4 |        333.1 |         170.6 |         1205.9 |    2.67x |
------------------------------------------------------------------------------------------------------
```

#### Ball Count Scaling ($N = 4, 8, 16, 32, 64$ — 10,000,000 Runs)

| Balls ($N$) | Method | Mean (ns) | Median (ns) | Std Dev (ns) | Best 10% (ns) | Worst 10% (ns) |
| ---: | :--- | ---: | ---: | ---: | ---: | ---: |
| **4** | `Simple` | 39.8 | 42.9 | 11.5 | 31.4 | 50.9 |
| **4** | `DE (Constant)` | 48.6 | 51.5 | 13.8 | 36.4 | 62.4 |
| **4** | `DE (Euler)` | 49.2 | 52.0 | 14.0 | 36.7 | 63.5 |
| **4** | `DE (RK4)` | 51.6 | 53.8 | 15.0 | 38.0 | 71.0 |
| **4** | `Homotopy` | 94.1 | 79.6 | 90.2 | 56.5 | 303.8 |
| **8** | `Simple` | 66.2 | 64.1 | 17.4 | 48.4 | 88.2 |
| **8** | `DE (Constant)` | 82.4 | 76.8 | 29.2 | 62.4 | 109.5 |
| **8** | `DE (Euler)` | 84.0 | 76.5 | 33.1 | 57.8 | 122.1 |
| **8** | `DE (RK4)` | 90.3 | 81.0 | 39.5 | 62.9 | 133.6 |
| **8** | `Homotopy` | 190.4 | 118.2 | 200.4 | 88.0 | 664.7 |
| **16** | `Simple` | 152.6 | 154.8 | 30.8 | 122.1 | 188.3 |
| **16** | `DE (Constant)` | 180.9 | 171.7 | 48.2 | 145.8 | 241.2 |
| **16** | `DE (Euler)` | 185.6 | 174.4 | 53.1 | 149.9 | 247.1 |
| **16** | `DE (RK4)` | 196.0 | 197.8 | 99.9 | 152.4 | 278.8 |
| **16** | `Homotopy` | 407.6 | 221.0 | 350.2 | 177.6 | 1255.3 |
| **32** | `Simple` | 455.3 | 438.9 | 91.5 | 392.6 | 584.6 |
| **32** | `DE (Constant)` | 532.4 | 513.2 | 125.9 | 458.0 | 688.2 |
| **32** | `DE (Euler)` | 518.8 | 500.9 | 127.4 | 454.7 | 644.5 |
| **32** | `DE (RK4)` | 528.3 | 525.7 | 80.8 | 454.6 | 660.2 |
| **32** | `Homotopy` | 941.0 | 897.0 | 531.1 | 501.3 | 2145.1 |
| **64** | `Simple` | 1684.0 | 1649.1 | 221.3 | 1574.6 | 2110.0 |
| **64** | `DE (Constant)` | 1800.1 | 1759.0 | 247.2 | 1675.7 | 2217.5 |
| **64** | `DE (Euler)` | 1808.8 | 1772.0 | 187.7 | 1689.4 | 2177.9 |
| **64** | `DE (RK4)` | 1818.5 | 1778.1 | 215.9 | 1680.2 | 2246.5 |
| **64** | `Homotopy` | 2626.6 | 2344.2 | 774.7 | 1792.2 | 4282.1 |

#### Operational Scenarios (10,000,000 Runs)

| Scenario | Method | Mean (ns) | Median (ns) | Std Dev (ns) | Best 10% (ns) | Worst 10% (ns) |
| :--- | :--- | ---: | ---: | ---: | ---: | ---: |
| **Clear LOS** | `Simple` | 152.3 | 154.0 | 100.9 | 119.3 | 201.6 |
| **Clear LOS** | `DE (Constant)` | 179.4 | 170.6 | 35.8 | 152.9 | 224.1 |
| **Clear LOS** | `DE (Euler)` | 185.5 | 172.0 | 71.7 | 153.9 | 251.2 |
| **Clear LOS** | `DE (RK4)` | 196.8 | 198.7 | 44.8 | 160.8 | 266.2 |
| **Clear LOS** | `Homotopy` | 409.7 | 221.1 | 345.3 | 185.1 | 1250.6 |
| **Blocked LOS** | `Simple` | 152.2 | 153.8 | 28.0 | 124.1 | 183.6 |
| **Blocked LOS** | `DE (Constant)` | 178.7 | 169.7 | 36.6 | 153.0 | 222.9 |
| **Blocked LOS** | `DE (Euler)` | 188.2 | 175.7 | 67.4 | 154.8 | 253.4 |
| **Blocked LOS** | `DE (RK4)` | 190.4 | 190.3 | 46.0 | 156.1 | 254.9 |
| **Blocked LOS** | `Homotopy` | 404.2 | 221.4 | 342.3 | 177.6 | 1236.9 |
| **Dense (32 Balls)** | `Simple` | 457.5 | 440.3 | 116.3 | 393.9 | 587.6 |
| **Dense (32 Balls)** | `DE (Constant)` | 518.4 | 499.1 | 121.8 | 448.1 | 673.9 |
| **Dense (32 Balls)** | `DE (Euler)` | 523.6 | 503.5 | 138.8 | 451.5 | 680.4 |
| **Dense (32 Balls)** | `DE (RK4)` | 529.6 | 529.3 | 95.1 | 455.9 | 656.8 |
| **Dense (32 Balls)** | `Homotopy` | 935.5 | 892.0 | 532.4 | 499.2 | 2137.9 |
| **Moving LOS** | `Simple` | 153.1 | 154.7 | 34.4 | 122.8 | 189.6 |
| **Moving LOS** | `DE (Constant)` | 180.6 | 170.6 | 43.8 | 152.5 | 229.6 |
| **Moving LOS** | `DE (Euler)` | 182.6 | 171.3 | 45.3 | 153.8 | 234.0 |
| **Moving LOS** | `DE (RK4)` | 195.4 | 199.8 | 48.7 | 161.9 | 256.7 |
| **Moving LOS** | `Homotopy` | 401.1 | 218.9 | 334.4 | 185.8 | 1220.8 |

![Update latency as the number of balls increases](experiments/results_ball_scaling.svg)

![Update latency across clear, blocked, dense, and moving-line-of-sight scenes](experiments/results_scenarios.svg)

![Summary update latency distribution for the 16-ball scene](experiments/results_summary.svg)

![Fast-path hit rate across operational scenarios](experiments/results_fastpath.svg)

![Fast-path hit rate vs timestep dt](experiments/results_low_dt_fastpath.svg)

![Approximation error (RMSE) vs timestep dt](experiments/results_low_dt_accuracy.svg)

The methods solve the same geometric first-hit problem, so their tracked paths nearly overlap. In the 600-frame experiment, each DE and Homotopy path differs from the Simple reference by at most about 0.003 pixels. The experiment CSV records the hit type for every frame.

At about 1.12 seconds, the first-hit point jumps because the moving blocker leaves the cue ray. The closest intersection then changes from that blocker to the selected target ball. This geometric change occurs in Simple and all three DE variants as well as Homotopy; it is not a numerical blow-up. The plots break the lines at blocker-to-target switches so the transition does not look like a continuous trajectory.

#### Tracked first-hit point over time

Each plot shows one method against the Simple reference. The x and y coordinates are shown in separate panels; line breaks mark a change between hitting the blocker and hitting the target.

![DE 0th tracked first-hit point](experiments/tracked_point_de_0th.svg)

![DE Euler tracked first-hit point](experiments/tracked_point_de_euler.svg)

![DE RK4 tracked first-hit point](experiments/tracked_point_de_rk4.svg)

![Homotopy tracked first-hit point](experiments/tracked_point_homotopy.svg)

Simple is expected to do well here: it evaluates inexpensive closed-form line-circle intersections in only two dimensions. Higher-order predictors and continuation methods cannot remove the work of considering the candidate balls, and they add their own numerical work. In higher dimensions, with more complicated geometric constraints, or with polynomial systems that lack a cheap direct solution, methods such as predictor-corrector continuation or Homotopy can become more useful and may outperform a direct approach. That depends on the problem and should be measured rather than assumed.

### Why Each Method Is Faster or Slower: Algorithmic, Statistical & Hardware Breakdown

With `@inline`, `@fastmath`, and `-O3 -g0` enabled across 10,000,000 runs, the latency hierarchy follows the exact arithmetic cost of each method (`146.6 ns` for `Simple`, `176.9 ns` for `DE (Constant)`, `183.3 ns` for `DE (Euler)`, `192.0 ns` for `DE (RK4)`, and `391.1 ns` for `Homotopy`), while the three DE solvers remain tightly clustered within **`1.21x–1.31x`** of `Simple` (`~15 ns` spread between `Constant`, `Euler`, and `RK4`). This tight clustering is a direct consequence of the trade-off between **register-inlined FMA arithmetic** and **fast-path / continuation convergence rates**:

| Method | Fast-Path Predictor Cost | Fast-Path Corrector Cost | $2\times 2$ Linear Solves / Fast-Path | Fast-Path Hit Rate (`Moving LOS`) | Central Loop Continuation Calls (10M) | Std Dev (`Summary`) | Worst 10% (`Summary`) |
| :--- | :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| **`Simple`** | None (stateless) | Exact `sqrt(R² - d_⊥²)` | **0** | N/A | 0 (direct `sqrt`) | 37.5 ns | 177.4 ns |
| **`DE (Constant)`** | 2 additions (`x + Δx`) | 1× `newton_step` | **1** | 88.79% | 546,582 | 41.2 ns | 222.5 ns |
| **`DE (Euler)`** | 1× `euler_step` (`dx_dt`) | 1× `newton_step` | **2** | 93.36% | 406,356 | 38.4 ns | 231.8 ns |
| **`DE (RK4)`** | 4-stage `rk4_step` | 1× `newton_step` | **5** | 98.60% | 245,490 | 48.4 ns | 255.5 ns |
| **`Homotopy`** | None (stateless) | Adaptive RK4+Newton on 2 roots | **~42 / ball** | 0% (no fast path) | Every candidate ball | **333.1 ns** | **1205.9 ns** |

#### Statistical Distribution Analysis: Mean vs. Median, Std Dev & Tail Latency (Best 10% vs. Worst 10%)

1. **Why `Homotopy` Has a Low Median (`216.4 ns`) but Huge Mean (`391.1 ns`), Std Dev (`333.1 ns`), and Worst 10% (`1205.9 ns`)**:
   - In the 16-ball scene, **71.8%** of frames have an unobstructed line of sight (`d_perp_sq >= R²` for all balls). On those unobstructed frames, `Homotopy` performs only the broad-phase dot-product checks, yielding a **Best 10% of `170.6 ns`** and a **Median of `216.4 ns`**—very close to the DE solvers.
   - However, on the **28.2%** of frames where one or more balls block the sightline, `Homotopy` has no temporal fast path (`0%`) and tracks *both* polynomial roots (`r_entry` and `r_exit`) through ~4 adaptive RK4 + Newton steps (~42 linear $2\times 2$ solves per blocking ball). Consequently, its **Worst 10% spikes to `1205.9 ns`**—**5.6× slower than its Median**—creating a heavily right-skewed distribution with **8× higher standard deviation (`333.1 ns` vs. `38.4–48.4 ns`)**.
2. **Why the DE Methods Maintain Low Standard Deviation (`38.4–48.4 ns`) and Tight Worst 10% (`222.5–255.5 ns`)**:
   - Because the temporal fast path resolves **88.79% (`Constant`)**, **93.36% (`Euler`)**, and **98.60% (`RK4`)** of blocked frames in a single predictor-corrector step *before* the central loop and narrows `t_min` immediately, blocked frames cost nearly the same as unobstructed frames.
   - As a result, all three DE methods have compact, near-symmetric distributions where **Mean ≈ Median** (`168.2–195.5 ns`) and their **Worst 10% (`222.5–255.5 ns`)** is **4.7× to 5.4× faster** than `Homotopy`'s Worst 10% (`1205.9 ns`), avoiding frame-time spikes when balls cross the cue ray.
3. **Why `-O3 -g0` + `@inline @fastmath` Keeps `DE (Constant)`, `DE (Euler)`, and `DE (RK4)` Within `~15 ns` (`176.9 ns`, `183.3 ns`, `192.0 ns`)**:
   - With `@inline @fastmath` under `-O3 -g0`, LLVM keeps all 2-tuples (`x`, `p`, `n`, `c`, `J`) in scalar AVX registers and fuses `solve_2x2` into hardware FMA instructions. Each additional predictor order adds only **`~6.4 ns`** (`Constant` $\to$ `Euler`) and **`~8.7 ns`** (`Euler` $\to$ `RK4`) on average.
   - Much of the extra arithmetic in `euler_step` (1 extra `solve_2x2`) and `rk4_step` (4 extra `solve_2x2` stages) is amortized by their higher fast-path hit rates (`93.36%` and `98.60%` vs. `88.79%`), which cut central-loop continuation calls by **26%** (`546,582 → 406,356` in `Euler`) and **55%** (`546,582 → 245,490` in `RK4`, with **19× fewer `sqrt` fallbacks**: `26,774` vs. `517,986`).

#### Function Call Breakdown: With Fast-Path vs. Without Fast-Path (Central Loop)

To see concretely why each method has a different computational cost, below is the exact sequence of functions invoked per candidate ball when a collision is resolved **via the Temporal Fast-Path (`tracked_fast == true`)** versus **via the Central Loop (`tracked_fast == false`)**:

| Method | Path A: Temporal Fast-Path (`prev_ball_id != 0`, before loop) | Path B: Central Loop (`for j in 1:num_balls`, when `d_perp_sq < R²`) |
| :--- | :--- | :--- |
| **`Simple`**<br>(`SimpleSolution.jl`) | *(No fast path — stateless across frames)* | 1. Inline `sqrt(rad_sq - d_perp_sq)` *(1× hardware `sqrtss`)*<br>2. Inline ray point `qx + t_hit * ux`<br>**Total: 0 function calls, 0 `solve_2x2`, 1 `sqrt`** |
| **`DE (Constant)`**<br>(`DEConstantSolution.jl` $\to$ `DESolution.jl`) | 1. Inline predictor: `x_pred = prev_sol .+ PREV_DELTA`<br>2. `newton_step(x_pred, p, n, c_now, rad)` *(1 iter)*:<br>&nbsp;&nbsp;• `f_error` $\to$ `jacobian` $\to$ `solve_2x2`<br>3. `f_error(x_corr, ...)` *(convergence check)*<br>4. Central loop skips `j == best_hit_id`<br>**Total: 2× `f_error`, 1× `jacobian`, 1× `solve_2x2`** | 1. `track_continuation(r_entry, ..., c_now, rad)` (`:zero_order`):<br>&nbsp;&nbsp;• `newton_step(r_entry, p, n, c_next, rad)` *(1 iter)*:<br>&nbsp;&nbsp;&nbsp;&nbsp;↳ `f_error` $\to$ `jacobian` $\to$ `solve_2x2`<br>2. `f_error(sol1, ...)` *(residual check `< 1e-4`)*<br>3. If `!found_hit` (~94.8% of loop calls):<br>&nbsp;&nbsp;• Fallback `sqrt(max(0.0, rad_sq - d_perp_sq))`<br>**Total: 2× `f_error`, 1× `jacobian`, 1× `solve_2x2` (+ 1× `sqrt` on fallback)** |
| **`DE (Euler)`**<br>(`DEEulerSolution.jl` $\to$ `DESolution.jl`) | 1. `euler_step(prev_sol, p, dp_dt, n_prev, dn_dt, c_prev, dc_dt, dt)`:<br>&nbsp;&nbsp;• `dx_dt`: `jacobian` $\to$ `df_dt` $\to$ `solve_2x2`<br>2. `newton_step(x_pred, p, n, c_now, rad)` *(1 iter)*:<br>&nbsp;&nbsp;• `f_error` $\to$ `jacobian` $\to$ `solve_2x2`<br>3. `f_error(x_corr, ...)` *(convergence check)*<br>4. Central loop skips `j == best_hit_id`<br>**Total: 1× `df_dt`, 2× `f_error`, 2× `jacobian`, 2× `solve_2x2`** | 1. `track_continuation(r_entry, ..., c_now, rad)` (`:euler`):<br>&nbsp;&nbsp;• `jacobian(x_curr, n, c_mid)` $\to$ `solve_2x2(J, f_s)` *(midpoint Euler predictor)*<br>&nbsp;&nbsp;• `newton_step(x_pred, p, n, c_next, rad)` *(1 iter)*:<br>&nbsp;&nbsp;&nbsp;&nbsp;↳ `f_error` $\to$ `jacobian` $\to$ `solve_2x2`<br>2. `f_error(sol1, ...)` *(residual check `< 1e-4`)*<br>3. If `!found_hit` (~73.4% of loop calls):<br>&nbsp;&nbsp;• Fallback `sqrt(max(0.0, rad_sq - d_perp_sq))`<br>**Total: 2× `f_error`, 2× `jacobian`, 2× `solve_2x2` (+ 1× `sqrt` on fallback)** |
| **`DE (RK4)`**<br>(`DERK4Solution.jl` $\to$ `DESolution.jl`) | 1. `rk4_step(prev_sol, p, dp_dt, n_prev, dn_dt, c_prev, dc_dt, dt)`:<br>&nbsp;&nbsp;• `k1 = dx_dt(...)`: `jacobian` $\to$ `df_dt` $\to$ `solve_2x2`<br>&nbsp;&nbsp;• `k2 = dx_dt(...)`: `jacobian` $\to$ `df_dt` $\to$ `solve_2x2`<br>&nbsp;&nbsp;• `k3 = dx_dt(...)`: `jacobian` $\to$ `df_dt` $\to$ `solve_2x2`<br>&nbsp;&nbsp;• `k4 = dx_dt(...)`: `jacobian` $\to$ `df_dt` $\to$ `solve_2x2`<br>2. `newton_step(x_pred, p, n, c_now, rad)` *(1 iter)*:<br>&nbsp;&nbsp;• `f_error` $\to$ `jacobian` $\to$ `solve_2x2`<br>3. `f_error(x_corr, ...)` *(convergence check)*<br>4. Central loop skips `j == best_hit_id`<br>**Total: 4× `df_dt`, 2× `f_error`, 5× `jacobian`, 5× `solve_2x2`** | 1. `track_continuation(r_entry, ..., c_now, rad)` (`:rk4`):<br>&nbsp;&nbsp;• `J1 = jacobian` $\to$ `solve_2x2(J1, f_s1)` (`k1`)<br>&nbsp;&nbsp;• `J2 = jacobian` $\to$ `solve_2x2(J2, f_s2)` (`k2`)<br>&nbsp;&nbsp;• `J3 = jacobian` $\to$ `solve_2x2(J3, f_s3)` (`k3`)<br>&nbsp;&nbsp;• `J4 = jacobian` $\to$ `solve_2x2(J4, f_s4)` (`k4`)<br>&nbsp;&nbsp;• `newton_step(x_pred, p, n, c_next, rad)` *(1 iter)*:<br>&nbsp;&nbsp;&nbsp;&nbsp;↳ `f_error` $\to$ `jacobian` $\to$ `solve_2x2`<br>2. `f_error(sol1, ...)` *(residual check `< 1e-4`)*<br>3. If `!found_hit` (only ~10.9% of loop calls):<br>&nbsp;&nbsp;• Fallback `sqrt(max(0.0, rad_sq - d_perp_sq))`<br>**Total: 2× `f_error`, 5× `jacobian`, 5× `solve_2x2` (rarely needs `sqrt`)** |
| **`Homotopy`**<br>(`HomotopySolution.jl`) | *(No fast path — stateless across frames)* | For **both** `r_entry` and `r_exit` (2× calls to `track_fast`):<br>1. Adaptive loop `while t < 1.0` (~4 sub-steps: $\Delta t = 0.1, 0.2, 0.4, 0.3$):<br>&nbsp;&nbsp;• `rk4_step_fast`: 4 stages $\times$ (`Hx_fast` $\to$ `dh1dx_fast` + `dh2dx_fast`, `Ht_fast` $\to$ `fn_fast`, `solve_2x2_fast`)<br>&nbsp;&nbsp;• `newton_step_fast` (1–2 iters): `Hx_fast`, `H_fast` $\to$ `fn_fast`, `solve_2x2_fast`<br>2. Final polishing `newton_step_fast(curr, 1.0, pi, pf)`<br>3. `fn_fast(sol, pf)` on both `(sol1, sol2)`<br>**Total (for 2 roots): ~42× `Hx_fast`, ~44× `fn_fast`, ~42× `solve_2x2_fast`** |

**Key Takeaway from Comparing Path A vs. Path B:**
- When a DE method takes **Path A (Fast-Path)**, it resolves the active blocker **before** the loop and immediately shrinks `t_min` to that blocker's distance. Inside the central loop (`for j in 1:num_balls`), it skips `j == best_hit_id` (`tracked_fast && j == best_hit_id && continue`) and uses the tighter `t_min` to prune any balls behind the blocker via the cheap axial check (`t_proj >= t_min + rad`), avoiding `track_continuation` (Path B) entirely unless a *new* closer ball enters the ray.
- When **Path A fails** (or on the first frame of a new blocker), execution falls through to **Path B** in the central loop, paying both the failed Path A cost *and* the Path B `track_continuation` cost (plus the fallback `sqrt` when `track_continuation` does not converge). This explains why `DE (Euler)` and `DE (RK4)`—which fail Path A `1.7×` and `8.0×` less often than `DE (Constant)`—stay within just `~6–15 ns` of `DE (Constant)` when compiled with `-O3 -g0` and `@inline @fastmath`.

### Detailed Experimental Investigations (10,000,000 Runs)

#### 1. Central Loop Fast-Path Frequency (`tracked_fast`)
In `DESolution.aim_cue!`, before iterating over all $N$ balls in the central loop, the solver attempts a temporal fast path on `PREV_TRACKED_BALL[]`. When `newton_move_sq < 0.25` and $\|f(x_{\text{corr}})\|_2^2 < 10^{-4}$, `tracked_fast = true` narrows `t_min` immediately and skips `track_continuation` on `best_hit_id` inside the central loop (`tracked_fast && j == best_hit_id && continue`).

Across **10,000,000 simulated updates** in the `Moving LOS` / `Standard 16 Balls` benchmark (`2,821,456` frames with an active blocker, i.e., `28.2%` of all frames):
- **`DE (Constant)`**: takes the fast path on **2,505,054 / 2,821,456** blocked frames (**88.79%** of eligible frames, **25.05%** of all frames), calling `track_continuation` `546,582` times in the central loop (`28,596` converged, `517,986` exact fallbacks).
- **`DE (Euler)`**: takes the fast path on **2,634,112 / 2,821,456** blocked frames (**93.36%** of eligible frames, **26.34%** of all frames), calling `track_continuation` `406,356` times (`108,088` converged, `298,268` exact fallbacks).
- **`DE (RK4)`**: takes the fast path on **2,782,037 / 2,821,458** blocked frames (**98.60%** of eligible frames, **27.82%** of all frames), calling `track_continuation` only `245,490` times (with an `89.1%` continuation convergence rate: `218,716` converged, needing the exact fallback only `26,774` times across `160,000,000` loop iterations).

#### 2. Timestep Scaling Experiments ($\Delta t$ from $1/1\text{ s}$ down to $1/1200\text{ s}$)
Decreasing $\Delta t$ from $1/1\text{ s}$ ($1.0\text{ s}$) down to $1/1200\text{ s}$ ($8.33 \times 10^{-4}\text{ s}$) reduces per-step ball displacement $\|v \Delta t\|$ and chord curvature error $O(\Delta t^2)$:
- **Fast-Path Convergence**: At very large timesteps ($\Delta t = 1/1\text{ s}$), the fast path is rejected (`0.00%`) across all methods; by $\Delta t = 1/10\text{ s}$, `DE (RK4)` already reaches **91.67%** while `Constant` and `Euler` remain at `0.00%`; at $\Delta t = 1/30\text{ s}$, `DE (Euler)` (**79.41%**) pulls ahead of `DE (Constant)` (**61.76%**); and as $\Delta t$ shrinks to $1/1200\text{ s}$, all three methods converge to **99.93%**.
- **Accuracy Scaling**: Against the exact `Float64` reference, `DE (RK4)` maintains a flat floor of $\text{RMSE} \approx 1.3 \times 10^{-5}\text{ px}$ across all $\Delta t$, while `DE (Constant)` and `DE (Euler)` improve steadily from $\text{RMSE} \approx 7.14 \times 10^{-4}\text{ px}$ at $1/1\text{ s}$ down to $2.5 \times 10^{-5}\text{ px}$ and $2.4 \times 10^{-5}\text{ px}$ at $1/1200\text{ s}$.

#### 3. Constant Predictor Newton Iteration Ablation (2 vs. 1 vs. 0 Iterations)
We validated the impact of reducing the number of Newton-Raphson corrector iterations in `DE (Constant)` (`newton_step` strictly executing a single iteration) as well as eliminating the redundant second `track_continuation` call on `r_exit`:
- **2 → 1 Newton Iteration**: At $\Delta t = 1/120\text{ s}$, reducing from `2` Newton iterations to `1` preserves identical maximum error vs. `Simple` (`0.002458 px`) and virtually identical RMSE (`0.000284 px` vs. `0.000270 px`, or `0.000127 px` vs. `0.000110 px` against exact `Float64`), while maintaining a **96.27%** fast-path hit rate and cutting update latency from `~220.8 ns` to **`~146.2 ns`**.
- **1 → 0 Newton Iterations (Predictor-Only)**: Removing the remaining Newton corrector step from the fast path (`0` iterations, pure linear extrapolation `x_corr = prev_sol + PREV_DELTA`) causes $\|f(x_{\text{corr}})\|_2^2 \ge 10^{-4}$ on every frame at $\Delta t = 1/120\text{ s}$ (**0.00%** fast-path hit rate, forcing 100% fallback to the central loop; if the residual check is also disabled, the uncorrected tangent extrapolation diverges to `374.0 px` error). At very small timesteps ($\Delta t = 1.67 \times 10^{-4}\text{ s}$), even `0` Newton iterations in the fast path achieves a **57.08%** fast-path hit rate with `RMSE = 0.000319 px`.

#### 4. Mathematical & Numerical Validation of `track_continuation`
Validating `DESolution.track_continuation` uncovered and resolved three critical issues:
1. **Zero-Slope Degeneracy at $s = 0$ ($\partial f_2 / \partial s|_{s=0} \equiv 0$)**: Because `c_start = p + t_proj * u` is the orthogonal projection of `c_target` onto the ray, the center shift $\Delta c = c_{\text{target}} - c_{\text{start}}$ is strictly orthogonal to $u$, while the initial root offset $\text{ic} - c_{\text{start}} = \mp R u$ is strictly parallel to $u$. Consequently, $(\text{ic} - c_{\text{start}}) \cdot \Delta c \equiv 0$, which previously caused the naive derivative $\partial f_2 / \partial s = -2(x - c_{\text{start}}) \cdot \Delta c$ at $s = 0$ to evaluate to **exact zero** (`(0.0, 0.0)`), making `:euler` predict zero displacement (`x_pred == ic`, identical to `:zero_order`) and zeroing stage `k1` of `:rk4`. Using the convex-linear-homotopy / midpoint derivative $\partial H_2 / \partial s = -2(x - c_{\text{mid}}) \cdot \Delta c$ eliminates this degeneracy, improving `:euler` continuation accuracy by up to $10^6\times$ (convergence rate across $d_\perp \in [0, R)$ rising from **10.6% → 45.0%** for 1 step and **62.2%** for 4 steps) and `:rk4` residual at $d_\perp = 10\text{ px}$ by $17,000\times$ ($6.02 \times 10^{-5} \to 3.56 \times 10^{-9}$, with single-step convergence rising from **80.0% → 97.8%** of the ball radius).
2. **Multi-Step Loop Restoration (`steps > 1`)**: Restored the `for _ in 1:steps` continuation loop across `:zero_order`, `:euler`, and `:rk4` so multi-step homotopy continuation properly integrates from $s = 0$ to $s = 1$.
3. **Single-Root `r_entry` Tracking**: Removed the redundant `track_continuation(r_exit, ...)` call in `aim_cue!`, halving continuation work per candidate ball in the central loop without affecting accuracy.

The benchmark renders SVG directly. If `rsvg-convert` is installed, it also creates PNG copies.


