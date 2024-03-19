using ForwardDiff
# abstract type CMDP{S,A} <: MDP{S,A} end
abstract type DiffPOMDP{S, A, O} <: POMDPs.POMDP{S, A, O} end

# on problem side Q and R on the solver side make Lambda (from cost function)
# remove project.toml and have using DiffPOMDPs.src.diffpomdp (or something like that)
# make sure same imports as top level 

# correct way is to make a new Project.toml for diffpomdps (pkg generate module)
"""
transition_mean(p::DiffPOMDP, state, action)

Returns mean next stat function
"""
function transition_mean(p::DiffPOMDP, state, action) end # dynamics function f
function observation_mean(p::DiffPOMDP, state) end # measurement function g
function transition_cov end 
function observation_cov end

function transition(p::DiffPOMDP, state, action)

    # before jacobian: x_t+1 = f(xt, ut)

    # after jacobian: x_t+1 = f(xt, ut) + A_t(xt - m_t)
    A, _ = jacobians(p, currentobs(b), action)

    ## 
    # make a distribution using transition_mean(p,state, action) and transition_Sigma(p, state, action)

    # paper said deterministic dynamics 
    return A*(state-currentobs(b)) + transition_mean(p, currentobs(b), action) 
end

function observation(p::DiffPOMDP, action, statep)
    _, C = jacobians(p, currentobs(b), action)
    return Normal(C*(statep-currentobs(b))+observation_mean(p, currentobs(b)),observation_cov)
end

# functions f and g should be defined when DiffPOMDP is defined (and maybe noise w with measurement?)


function jacobians(p, b::GaussianBelief, a)
    # compute Jacobian A matrix 

    # A _t = df/dx (m_t, u_t)
    # f is the process dynamics, x_{t+1} = f(x_t, u_t) 
    # A = ForwardDiff.jacobian(transition_mean(p, currentobs(b), a))
    # A = ForwardDiff.jacobian(p.transition_mean, [b.mean, a])
    tm(v) = [v[1], v[2]]; # transition mean, v[1] = x, v[2] = u 
    # A = ForwardDiff.jacobian(tm, [b.μ, a])
    A = [1 0 1 0; 1 0 1 0]

    # C_t = dg/dx (m_t)
    # g is the deterministic component of measurement function, z_t = g(x_t) + w
    om(v) = [v[1] + p.observation_cov, 0]; # observation mean, v[1] = x
    # C = ForwardDiff.jacobian(om, b.μ)
    C = [1 0; 0 1]
    
    return A, C
end 









# function diffO(d::DiffPOMDP, s, a, sp)
#     # observation(m::POMDP, action, statep)
#     # observation(m::POMDP, state, action, statep)

#     # Return the observation distribution 

#     # using POMDPModelTools # for SparseCat

#     # struct MyPOMDP <: POMDP{Int, Int, Int} end

#     # observation(p::MyPOMDP, sp::Int) = SparseCat([sp-1, sp, sp+1], [0.1, 0.8, 0.1])
#     _ , C = jacobians(d, b, a)
#     return C(s-b.m) + d.g(b.m) + d.w # o
# end

# """

#     diffT(p::DiffPOMDP, s, a, o)

# Return the transition function for the differentiable POMDP.
    
# """

# function diffT(d::DiffPOMDP, s, a)
#     # transition(m::POMDP, state, action) 

#     # Return the transition distribution from the current state-action pair

#     A,_ = jacobians(d, b, a)
#     return A(s-b.m) + d.f(b.m, a) # sp

# end

