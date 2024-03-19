# using GaussianBelief
using GaussianFilters
using POMDPs
using POMDPTools
# using .BeliefiLQR

include("/Users/michelleho24/Documents/GitHub/aa229_proj/DiffPOMDPs.jl/src/DiffPOMDPs.jl")
include("/Users/michelleho24/Documents/GitHub/aa229_proj/BeliefiLQR.jl/src/BeliefiLQR.jl")

struct LightDarkState
    # status::Int64
    x::Float64
    y::Float64
end

struct LightDarkAction
    u_x::Float64
    u_y::Float64
end

struct LightDarkObservation
    z_x::Float64
    z_y::Float64
end

mutable struct LightDark2D{S,A,O} <: DiffPOMDP{LightDarkState, LightDarkAction, LightDarkObservation}
    states::S
    actions::A
    observations::O
    transition_mean::Function
    transition_cov::Function
    observation_mean::Function
    observation_cov::Function
    reward::Function
end

# Q = [0.5 0; 0 0.5]
# R =[0.5 0; 0 0.5]
# Λ = [0.5 0; 0 0.5]
Q = 0.5
R = 0.5
Λ = 0.5
# A =  
goal_pos = [0.0, 0.0]
time_horizon = 10

default_sigma(x::Float64) = abs(x - 5)/sqrt(2) + 1e-2

discount(p::LightDark2D) = 0.9

isterminal(::LightDark2D, act::LightDarkAction) = act == 0

isterminal(::LightDark2D, s::LightDarkState) = s == [10,10]

observation(p::LightDark2D, sp::LightDarkState) = Normal(sp.y, p.sigma(sp.y))

function transition_mean(p::LightDark2D, s::LightDarkState, a::LightDarkAction)
    return s + a
    
function transition_cov(p::LightDark2D, s::LightDarkState, a::LightDarkAction)
    return 0
end

function observation_mean(p::LightDark2D, s::LightDarkState, a::LightDarkAction)
    return s
end

function observation_cov(p::LightDark2D, s::LightDarkState, a::LightDarkAction)
    return 0.5*(5-s.x)^2
end

function reward(p::LightDark2D, s::LightDarkState, u::LightDarkAction)
    if s.x == 0 && s.y == 0
        return 10
    elseif a == 0
            if abs(s.y) < 1
                return 0.1
            else
                return -0.1
            end
        else
            movement_cost = 0.1
            return -movement_cost*a
        end
    end
end

pomdp = LightDark2D(1:10, -1:1, 1:10, transition_mean, transition_cov, observation_mean, observation_cov, reward)

# initialize belief 
b = GaussianBelief([2.0, 2.0], [0.5 0; 0 0.5])
# initial action
u = [0.0, 0.0] 

solver = BeliefiLQR.BeliefiLQRSolver(Q, R, Λ, goal_pos, time_horizon)
# policy = solve(solver, pomdp)

U_traj = []
B_traj = []
# push!(B_traj, b)
costs = []

for i in 1:solver.time_horizon
    print("Time step", i)
    global b 
    global u
    # take action
    tf = solver.time_horizon - i
    U, action_info = BeliefiLQR.action_info(pomdp, solver, b, u, tf)
    b = action_info[:Beliefs][1]
    u = U

    mu_x = b[1]
    mu_y = b[2]
    sigma = [b[3], b[4]]
    s1, s2 = sigma[1], sigma[2]

    b0 = GaussianBelief([mu_x, mu_y], diagm([s1, s2]))

    # store action and belief
    push!(U_traj, U)
    push!(B_traj, b)

    # Sigma0 = 0.01*Matrix{Float64}(I,2,2)

    At, Ct = jacobians(pomdp, b, a)   

    A22 = -Ct'*(Ct*At*Sigma0*At'*Ct' + W)^-1*Ct*At*Sigma0*At'+ At*Sigma0*At'Ct'(I(2) + W)*Ct*At*Sigma0*At' -At*Sigma0*At'*Ct'*(Ct*At*Sigma0*At'*Ct' + W)^-1*Ct
    
    # dA = ForwardDiff.gradient()
    A21 = 0
    # A = [At, 0; A21, A22]
    A = At

    # B = [ForwardDiff.derivative(pomdp.transition_mean, [mu0, Sigma0], :u); 0]
    B = [1; 0]
    B = reshape(B, :, 1)

    # build dynamics model - linearized about the belief
    # A might not be the right dimensions 
    dmodel = LinearDynamicsModel(A,B,W);

    # nonlinear observation function. must be a function of both states (x) and actions (u) even if either are not used.
    C = Ct
    # V = Diagonal(pomdp.observation_cov())
    V = 0.01*Matrix{Float64}(I,2,2)
    # V = 0.3*Matrix{Float64}(I,3,3)

    # build observation model
    omodel = LinearObservationModel(C,V)

    # build ekf
    ekf = ExtendedKalmanFilter(dmodel,omodel);

    # Prediction
    bp = GaussianFilter.predict(ekf, b0, u)

    # Update: corrects this prediction using measurements
    z = pomdp.observation_mean(bp,u) + pomdp.observation_cov(bp,u)
    b_new = GaussianFilter.update(ekf, bp, z)
    b = [b_new.μ[1], b_new.μ[2], b_new.Σ[1][1], b_new.Σ[2][2]]

    # store cost
    push!(costs, reward(pomdp, b.μ, U))
end

# print cost and trajectory
print("Cost: ", sum(costs))
print("Trajectory Beliefs: ", B_traj)
print("Trajectory Actions: ", U_traj)
# plot trajectory
