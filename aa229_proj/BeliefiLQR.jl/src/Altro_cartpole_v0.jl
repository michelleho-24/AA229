# Set up problem using TrajectoryOptimization.jl and RobotZoo.jl
using TrajectoryOptimization
using Altro
import RobotZoo.Cartpole
using StaticArrays, LinearAlgebra
using RobotDynamics
const RD = RobotDynamics

# # Use the Cartpole model from RobotZoo
model = Cartpole()
n = RD.state_dim(model)
m = RD.control_dim(model)

# Define model discretization
N = 101
tf = 5.
dt = tf/(N-1)

# Define initial and final conditions
x0 = @SVector zeros(n)
xf = @SVector [5, 5, 0, 0]  # i.e. swing up

# Set up
Q = 1.0e-2*Diagonal(@SVector ones(n)) * dt
Qf = 100.0*Diagonal(@SVector ones(n))
R = 1.0e-1*Diagonal(@SVector ones(m)) * dt

function LQRObjective2(
    Q::Union{<:Diagonal, <:AbstractVector},
    R::Union{<:Diagonal, <:AbstractVector},
    Qf::Union{<:Diagonal, <:AbstractVector},
    xf::AbstractVector, N::Int;
    uf=(@SVector zeros(size(R,1))),
    checks=true)
n,m = size(Q,1), size(R,1)
@assert size(Q,1) == length(xf)
@assert size(Qf,1) == length(xf)
@assert size(R,1) == length(uf)

## TODO: add in \Lambda term 
Q,R,Qf = Diagonal(Q), Diagonal(R), Diagonal(Qf)
q = -Q*xf
r = -R*uf
c = 0.5*xf'*Q*xf + 0.5*uf'R*uf
qf = -Qf*xf
cf = 0.5*xf'*Qf*xf

ℓ = DiagonalCost(Q, R, q, r, c, checks=checks, terminal=false)

ℓN = DiagonalCost(Qf, R, qf, r, cf, checks=false, terminal=true)

diffmethod = RobotDynamics.DiffMethod[RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined()]

Objective(ℓ, ℓN, N, diffmethod=diffmethod)
end

obj = LQRObjective2(Q,R,Qf,xf,N)
# println(obj.diffmethod)

# Add constraints
conSet = ConstraintList(n,m,N)
u_bnd = 3.0
bnd = BoundConstraint(n,m, u_min=-u_bnd, u_max=u_bnd)
goal = GoalConstraint(xf)
add_constraint!(conSet, bnd, 1:N-1)
add_constraint!(conSet, goal, N)

# Initialization
u0 = @SVector fill(0.01,m)
U0 = [u0 for k = 1:N-1]

# Define problem
prob = Problem(model, obj, x0, tf, xf=xf, constraints=conSet)
initial_controls!(prob, U0)

# Solve with ALTRO
opts = SolverOptions(
    cost_tolerance_intermediate=1e-2,
    penalty_scaling=10.,
    penalty_initial=1.0
)
altro = Altro.iLQRSolver(prob, opts)
initial_controls!(altro, U0)
solve!(altro);
# altro = ALTROSolver(prob, opts)
# solve!(altro)

# Get some info on the solve
# max_violation(altro)  # 5.896e-7
# TrajectoryOptimization.cost(altro)           # 1.539
# iterations(altro)     # 44

# Extract the solution
X = TrajectoryOptimization.states(altro)
U = controls(altro)

# Extract the solver statistics
stats = Altro.stats(altro)   # alternatively, solver.stats
stats.iterations             # 44, equivalent to iterations(solver)
stats.iterations_outer       # 4 (number of Augmented Lagrangian iterations)
stats.iterations_pn          # 1 (number of projected newton iterations)
stats.cost[end]              # terminal cost
stats.c_max[end]             # terminal constraint satisfaction
stats.gradient[end]          # terminal gradient of the Lagrangian
dstats = Dict(stats)         # get the per-iteration stats as a dictionary (can be converted to DataFrame)
