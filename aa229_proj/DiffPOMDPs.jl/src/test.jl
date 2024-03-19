# using ForwardDiff

# # Define your function to take a vector as input
# f(x) = sin(x[1]^2) * cos(x[2])

# # Point at which you want to evaluate the derivative
# x_point = [1.0, 2.0]

# # Compute the derivative with respect to the first variable (x)
# df_dx = ForwardDiff.gradient(f, x_point)[1]

# # Compute the derivative with respect to the second variable
# df_dy = ForwardDiff.gradient(f, x_point)[2]

# println("df/dx at $x_point = $df_dx")
# println("df/dy at $x_point = $df_dy")

using ForwardDiff

# Define your function
f(x) = [x[1], x[2]]

# Point at which you want to evaluate the Jacobian
x_point = [1.0, 2.0]

# Compute the Jacobian matrix
J = ForwardDiff.jacobian(f, x_point)

println("Jacobian matrix at $x_point:")
println(J)

# using GaussianFilters, LinearAlgebra


# dt = 0.1
# m = 50
# A = [1 dt 0 0 ; 0 1 0 0 ; 0 0 1 dt; 0 0 0 1]
# B = [0 0; dt/m 0; 0 0; 0 dt/m]
# W = 0.1*Matrix{Float64}(I,4,4)

# # build linear dynamics model
# dmodel = LinearDynamicsModel(A,B,W);


# C = [0 1.0 0 0; 0 0 0 1.0]
# V = 0.5*Matrix{Float64}(I,2,2)

# # build linear observation model
# omodel = LinearObservationModel(C,V)

# # build kf
# kf = KalmanFilter(dmodel,omodel);