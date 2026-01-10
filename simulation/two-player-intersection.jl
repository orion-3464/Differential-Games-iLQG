using iLQGames
import iLQGames: dx
using StaticArrays
using LinearAlgebra
using Plots

include("src/dynamics/intersection.jl")
include("src/costs/intersection_costs.jl")

dynamics = Intersection()

costs = (
    FunctionPlayerCost(car1_cost),
    FunctionPlayerCost(car2_cost)
)

player_inputs = (
    SVector(1, 2),
    SVector(3, 4)
)

g = GeneralGame(game_horizon, player_inputs, dynamics, costs)

solver = iLQSolver(g)

x0 = SVector(
    # Car 1 (Compact)
    -2.5, 2.0, 0.0, 0.0, 0.0,
    # Car 2 (SUV)
    0.0, -2.5, π/2, 0.0, 0.0
)

converged, trajectory, strategies = solve(g, solver, x0)

if converged 
    println("Found Nash equilibrium. Plotting...")

    plt = plot_traj(trajectory, g, [:green, :red])
    savefig(plt, "results/intersection.png")

    @animated(plot_traj(trajectory, g, [:green, :red]),
          1:game_horizon, "results/intersection_animation.gif")

else
    println("Failed to converge. Aborting...")
end