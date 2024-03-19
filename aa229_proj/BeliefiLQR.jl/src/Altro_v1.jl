
using Altro
using LinearAlgebra
using TrajectoryOptimization
using RobotDynamics
using StaticArrays
using Random

# Define dynamics: double integrator
function dynamics(x̂, P, u)
    # Propagate the belief using the dynamics
    A = [1.0 1.0; 0.0 1.0]
    B = [0.0; 1.0]
    
    # Prediction step: propagate the mean and covariance
    x̂ = A * x̂ + B * u
    P = A * P * A' + eye(length(x̂))
    
    # EKF update
    H = [1.0 0.0]  # Measurement Jacobian
    R = 0.1 * eye(size(H, 1))  # Measurement covariance
    z = H * x̂ + sqrt(R) * randn(size(H, 1))  # Simulate noisy measurement
    y = z - H * x̂
    S = H * P * H' + R
    K = P * H' / S
    x̂ += K * y
    P = (eye(size(P)) - K * H) * P
    
    return x̂, P
end

# Define cost function
function cost(x̂, P, u)
    Q = [1.0 0.0; 0.0 1.0]
    R = 0.1
    return (x̂' * Q * x̂ + u' * R * u)[1]
end

# Define initial belief (mean and covariance)
x̂0 = [0.0, 0.0]
P0 = 0.1 * eye(length(x̂0))

# Define target state
xf = [1.0, 0.0]

# Define initial control sequence
U0 = zeros(1, 10)  # 10 time steps, single control input

# Create the optimization problem
prob = Problem(dynamics, cost, x̂0, P0, xf, U0)

# Set bounds on control inputs
umin = [-1.0]
umax = [1.0]
setbounds!(prob, umin, umax)

# Solve the problem using iLQR
opts = iLQRSolverOptions()
opts.max_iter = 100
opts.trust_shrink_ratio = 0.9
opts.trust_expand_ratio = 1.1

solver = iLQRSolver(prob, opts)
results = solve!(solver)

# Extract the optimized trajectory
X_opt = get_trajectory(results)
U_opt = get_control(results)

# Print optimized trajectory
println("Optimal trajectory:")
println("States: ", X_opt)
println("Controls: ", U_opt)
