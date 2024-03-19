using LightDarkPOMDPs
using Base.Test
using POMDPs
using POMDPToolbox
using StaticArrays
using Plots
using ParticleFilters

using POMDPs, 

# pomdp = LightDark2D(
#     states = 
#     actions = 
#     observations = 
#     initialstate =
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

solver = 
solver = QMDPSolver()
policy = solve(solver, m)

rsum = 0.0
for (s,b,a,o,r) in stepthrough(m, policy, "s,b,a,o,r", max_steps=10)
    println("s: $s, b: $([s=>pdf(b,s) for s in states(m)]), a: $a, o: $o")
    global rsum += r
end
println("Undiscounted reward was $rsum.")

# #### LightDark2D Problem ####

# p = LightDark2D()

# # simulate with a policy that moves -0.01 times the observation
# hr = HistoryRecorder(max_steps=10)
# h = sim(p, simulator=hr) do o
#     @show o
#     return -0.01*o
# end
# @show p.count

# # simulate with a kalman filter
# bpol = FunctionPolicy(b -> -0.5*mean(b))
# up = LightDark2DKalman(p)
# h = simulate(hr, p, bpol, up)

# plotly()
# plot(p)
# plot!(h)
# # gui()
