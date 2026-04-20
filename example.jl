# -------------------------------------------------------------------------- #
# The following example illustrates both direct and indirect solution 
# approaches for a constrained energy minimization problem. The workflow 
# demonstrates a practical strategy: a direct method on a coarse grid first 
# identifies the problem structure and provides an initial guess for the 
# indirect method, which then computes a solution up to arbitrary precision 
# via shooting based on Pontryagin's Maximum Principle.
# -------------------------------------------------------------------------- #

using Pkg
Pkg.activate(@__DIR__)
Pkg.instantiate()

# -------------------------------------------------------------------------- #
# Problem definition
# -------------------------------------------------------------------------- #

# Energy-optimal control with state constraint
using OptimalControl
t0 = 0
tf = 1
x0 = [-1, 0]
x_target = [0, 0]
v_max = 1.2
ocp = @def begin
    t ∈ [t0, tf], time
    x = (q, v) ∈ R², state
    u ∈ R, control
    x(t0) == x0              # initial condition
    x(tf) == x_target        # terminal condition
    v(t) ≤ v_max             # state constraint
    ∂(x)(t) == [v(t), u(t)]  # dynamics
    0.5∫(u(t)^2) → min       # minimize control energy
end

# -------------------------------------------------------------------------- #
# Direct method
# -------------------------------------------------------------------------- #

# The problem is transcribed to an NLP using a midpoint discretization 
# scheme and solved with an interior-point solver.
using NLPModelsIpopt
direct_sol = solve(ocp; grid_size=50)

# The initial costate and the two switching times are recovered from the 
# direct solution to initialize the indirect method.

t = time_grid(direct_sol)           # the time grid as a vector
x = state(direct_sol)               # the state as a function of time
p = costate(direct_sol)             # the costate as a function of time
p0 = p(t0)                          # initial costate
g(x) = v_max - x[2]                 # constraint: g(x) ≥ 0
I = findall(t -> g(x(t)) ≤ 1e-3, t) # times where constraint is active
t1 = t[first(I)]                    # entry time
t2 = t[last(I)]                     # exit time
initial_guess = [p0..., t1, t2]     # initial guess for shooting

# -------------------------------------------------------------------------- #
# Indirect method
# -------------------------------------------------------------------------- #

# The indirect method applies Pontryagin's Maximum Principle. 
# The solution has three phases (unconstrained-constrained-unconstrained 
# arcs), requiring definition of Hamiltonian flows for each phase and a 
# shooting function to enforce boundary conditions.

using OrdinaryDiffEq, NonlinearSolve

# Define Hamiltonian flows based on Pontryagin's Maximum Principle
f_interior = Flow(ocp, (x, p) -> p[2]) # u(x, p) is p[2]
f_boundary = Flow(ocp, (x, p) -> 0,    # u(x, p) is 0
                       (x, u) -> g(x), # when g(x) = 0 (constrained arc)
                       (x, p) -> p[1]) # associated multiplier is p[1]

# Shooting function
function shoot!(s, ξ, _) # unused last parameter (required by NonlinearSolve)
    p0, t1, t2 = ξ[1:2], ξ[3], ξ[4]
    x1, p1 = f_interior(t0, x0, p0, t1)  # flow on unconstrained arc [t0, t1]
    x2, p2 = f_boundary(t1, x1, p1, t2)  # flow on constrained arc [t1, t2]
    xf, pf = f_interior(t2, x2, p2, tf)  # flow on unconstrained arc [t2, tf]
    s[1:2] = xf - x_target               # terminal condition
    s[3] = g(x1)                         # entry condition on constraint
    s[4] = p1[2]                         # switching phase condition
end

# Solve shooting problem
shooting_sol = solve(NonlinearProblem(shoot!, initial_guess))
p0, t1, t2 = shooting_sol.u[1:2], shooting_sol.u[3], shooting_sol.u[4]

# -------------------------------------------------------------------------- #
# Trajectory reconstruction
# -------------------------------------------------------------------------- #

# The trajectory is reconstructed by concatenating the three flows using 
# the `*` operator, with the internal switching times `t1` and `t2`.
φ = f_interior * (t1, f_boundary) * (t2, f_interior)
indirect_sol = φ((t0, tf), x0, p0; saveat=range(t0, tf, 100))

# -------------------------------------------------------------------------- #
# Comparison
# -------------------------------------------------------------------------- #

# Compare both solutions visually and save the plot.
using Plots
plot(direct_sol; label="Direct")
plot!(indirect_sol; label="Indirect", color=2, linestyle=:dash)
savefig("comparison.pdf")