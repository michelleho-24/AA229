
# one step - must run in while loop to get from initial to goal state while b in not threshold of goal
module BeliefiLQR
    export BeliefiLQRSolver, Model, BiLQRpolicy, solve, action_info, LQRObjective2

    using RobotDynamics
    using LinearAlgebra
    using GaussianFilters
    using Altro
    using ForwardDiff
    using POMDPs
    using POMDPTools
    using RobotDynamics
    using StaticArrays

    using TrajectoryOptimization

    const RD = RobotDynamics

    include("/Users/michelleho24/Documents/GitHub/aa229_proj/DiffPOMDPs.jl/src/DiffPOMDPs.jl")

    # struct GaussianBelief
    #     mean::Vector{Float64},
    #     s1::Matrix{Float64},
    #     s2::Matrix{Float64}
    # end

    struct BeliefiLQRSolver <: POMDPs.Solver 
        Q::Float64
        R::Float64      
        Λ::Float64   
        goal_pos::Vector{Float64} 
        time_horizon::Int 
    end

    struct Model{T} <: RobotDynamics.ContinuousDynamics 
        mc::T
        function Model(mc)
            T = eltype(promote(mc))
            new{T}(mc)
        end
    end

    Model(; mc=1.0) = Model(mc) 

    abstract type BiLQRpolicy <: POMDPs.Policy end 
    # include solver, POMDP call action_info on this not the solver 
    # look at POMCPOW for example

    """
        solve(BeliefiLQRSolver::Solver, problem::DiffPOMDP)

    Solves the POMDP using method associated with solver, and returns a policy.
    """

    solve(solver::BeliefiLQRSolver, problem::DiffPOMDP) = BiLQRpolicy(solver, problem)

    """

    """

    # POMDPs.action(p::DiffPOMDP, s) = first(action_info(p, solver, b))

    """

    """


    function RobotDynamics.dynamics(model::Model, b, u)
        belief = GaussianFilters.GaussianBelief([b[1], b[2]], Diagonal([b[3], b[4]]))

        At, Ct = jacobians(pomdp, b, a)   
        # A = [[At 0], [0 At]]
        # A22 = -Ct'*(Ct*At*Sigma0*At'*Ct' + W)^-1*Ct*At*Sigma0*At'+ At*Sigma0*At'Ct'(I(2) + W)*Ct*At*Sigma0*At' -At*Sigma0*At'*Ct'*(Ct*At*Sigma0*At'*Ct' + W)^-1*Ct
        
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
        # ekf = ExtendedKalmanFilter(dmodel,omodel);
        kf = KalmanFilter(dmodel,omodel);
    
        # Prediction
        bp = GaussianFilters.predict(kf, belief, u)
    
        # Update: corrects this prediction using measurements
        z = pomdp.observation_mean(bp,u) + pomdp.observation_cov(bp,u)
        b_new = GaussianFilter.update(ekf, bp, z)
        return [b_new.μ[1], b_new.μ[2], b_new.Σ[1][1], b_new.Σ[2][2]]
    end
    
    function dynamics!(model::Model, xdot, b, u)
        xdot .= dynamics(model, b, u)
    end
    
    function LQRObjective2(
        Q_e, R, Λ, bf, N;
        uf=[0.0, 0.0],
        checks=true)
    
        muf = [bf[1], bf[2]]
        sigf = [bf[3], bf[4]]

        # print(muf)
    
        R,Λ = [R 0.0; 0.0 R], [Λ 0.0; 0.0 Λ]
        Q = [Q_e 0;0 0]
        q = -Q*muf

        r = -R*uf
        c = 0.5*muf'*Q*muf + 0.5*uf'*R*uf
        qf = -Λ*sigf
    
        cf = 0.5*sigf'*Λ*sigf
    
        ℓ = DiagonalCost(Q, R, q, r, c, checks=checks, terminal=false)
    
        ℓN = DiagonalCost(Λ, R, qf, r, cf, checks=false, terminal=true)
    
        diffmethod = RobotDynamics.DiffMethod[RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined(), RobotDynamics.UserDefined()]
    
        Objective(ℓ, ℓN, N, diffmethod=diffmethod)
    end
    
    RobotDynamics.state_dim(::Model) = 4
    RobotDynamics.control_dim(::Model) = 1
    

    function action_info(pomdp, solver, b::GaussianFilters.GaussianBelief, a, tf)

        # s = statetype(p.mdp)
        # local a::actiontype(p.mdp)
        info = Dict{Symbol, Any}()   

        # initialize 
        # μ0 = b.mean
        mu0 = b.μ
        Sigma0 = [b.Σ[1,1], b.Σ[2,2]]
        xf = solver.goal_pos
        # xf = [0,0]
        # W = Diagonal(pomdp.transition_cov(mu0,u))
        W = [0 0; 0 0]
        
        model = Model()
        n = RD.state_dim(model)
        m = RD.control_dim(model)

        # Set up
        # Q = 1.0e-2*Diagonal(@SVector ones(n)) * dt
        # Qf = 100.0*Diagonal(@SVector ones(n))
        # R = 1.0e-1*Diagonal(@SVector ones(m)) * dt
        Q = solver.Q
        R = solver.R
        Λ = solver.Λ

        # bf = GaussianFilters.GaussianBelief(muf, sigf)
        # xf = pomdp.goal_pos
        bf = [xf[1], xf[2], 0, 0]
        N = 101
        obj = LQRObjective2(Q,R,Λ,bf,N)

        # Initialization
        u0 = @SVector fill(0.01,m)
        U0 = [u0 for k = 1:N-1]

        # Define problem
        # Define problem

        b0 = @SVector [b.μ[1],b.μ[2], b.Σ[1,1], b.Σ[2,2]] # mu.x, mu.y, s1, s2
        prob = Problem(model, obj, b0, tf, xf=xf)
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

        # Extract the optimized trajectory
        Beliefs = TrajectoryOptimization.states(altro)
        U = controls(altro)

        # Print optimized trajectory
        println("Optimal trajectory:")
        println("States: ", X)
        println("Controls: ", U)

        stats = Altro.stats(altro)
        info[:iters] = stats.iterations
        info[:costs] = stats.cost
        
        info[:Beliefs] = Beliefs
        info[:U] = U

        return U[1], info

    end 

end 
