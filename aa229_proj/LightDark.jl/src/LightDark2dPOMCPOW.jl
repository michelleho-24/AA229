using POMDPs
using POMCPOW
using POMDPModels
using POMDPTools

solver = POMCPOWSolver(criterion=MaxUCB(20.0))
# TODO: need transition and observation mean and covs functionsfor lightdark
pomdp = # LightDark2D(
    #     states = 
    #     actions = 
    #     observations = 
    #     transition_mean = 
    #     transition_cov = 
    #     observation_mean = 
    #     observation_cov =
        # reward = function (s, a)
        #     if a == "listen"
        #         return -1.0
        #     elseif s == a # the tiger was found
        #         return -100.0
        #     else # the tiger was escaped
        #         return 10.0
        #     end
        # end
    # )
planner = solve(solver, pomdp)

##TODO: initial  belief? 

hr = HistoryRecorder(max_steps=100)
hist = simulate(hr, pomdp, planner)
for (s, b, a, r, sp, o) in hist
    @show s, a, r, sp
end

rhist = simulate(hr, pomdp, RandomPolicy(pomdp))
println("""
    Cumulative Discounted Reward (for 1 simulation)
        Random: $(discounted_reward(rhist))
        POMCPOW: $(discounted_reward(hist))
    """)