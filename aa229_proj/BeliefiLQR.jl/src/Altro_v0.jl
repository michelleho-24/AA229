using Altro
using LinearAlgebra
using TrajectoryOptimization
# using RobotDynamics
# using StaticArrays

# Define dynamics function for a single time step
function dynamics(x, u)
    A = [1.0 1.0; 0.0 1.0]
    B = [0.0; 1.0]
    return A * x + B * u
end

# Create the model vector by repeating the dynamics function
num_time_steps = 10
model = fill(dynamics, num_time_steps)

# Define cost function
function cost(x, u)
    Q = [1.0 0.0; 0.0 1.0]
    R = 0.1
    return (x' * Q * x + u' * R * u)[1]
end


# Define model discretization
N = 101
tf = 5.
dt = tf/(N-1)

# Define initial and final conditions
x0 = @SVector zeros(n)
xf = @SVector [0, pi, 0, 0]  # i.e. swing up

# Set up
Q = 1.0e-2*Diagonal(@SVector ones(n)) * dt
Qf = 100.0*Diagonal(@SVector ones(n))
R = 1.0e-1*Diagonal(@SVector ones(m)) * dt
obj = LQRObjective(Q,R,Qf,xf,N)

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

# # Define the cost function
# function cost(altro::Altro)
#     x = altro.x
#     u = altro.u
#     Q = [1.0 0.0; 0.0 1.0]
#     R = 0.1
#     return (x' * Q * x + u' * R * u)[1]
# end

# Define problem
prob = Problem(model, obj, x0, tf, xf=xf, constraints=conSet)
initial_controls!(prob, U0)

# Solve with ALTRO
altro = Altro.iLQRSolver(prob, opts)
initial_controls!(altro, U0)
solve!(altro);
# opts = SolverOptions(
#     cost_tolerance_intermediate=1e-2,
#     penalty_scaling=10.,
#     penalty_initial=1.0
# )
# altro = ALTROSolver(prob, opts)
# solve!(altro)

# Get some info on the solve
# max_violation(altro)  # 5.896e-7
# TrajectoryOptimization.cost(altro)           # 1.539
# iterations(altro)     # 44

# Extract the solution
X = states(altro)
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
