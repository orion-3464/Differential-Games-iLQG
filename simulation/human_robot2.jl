using Plots

include("src/dynamics/human_robot2_dynamics.jl")
include("src/costs/human_robot1_costs.jl")

dynamics = Game2()

costs = (
    FunctionPlayerCost(robot_cost),
    FunctionPlayerCost(human1_cost),
    FunctionPlayerCost(human2_cost)
)

player_inputs = (
    SVector(1, 2),
    SVector(3, 4),
    SVector(5, 6)
)

g = GeneralGame(game_horizon, player_inputs, dynamics, costs)

solver = iLQSolver(g)

x0 = SVector(
    # TurtleBot
    -3.0, -3.0, 0.0, 0.0,
    # Human1
    -3.0, 3.0, 0.0, 0.0,
    # Human2
    3.0, -3.0, 0.0, 0.0
)

converged, trajectory, strategies = solve(g, solver, x0)

if converged 
    println("Found Nash equilibrium. Plotting...")

    plt = plot_traj(trajectory, g, [:blue, :green, :red])
    savefig(plt, "results/robot_human1.png")

    @animated(plot_traj(trajectory, g, [:blue, :green, :red]),
          1:game_horizon, "results/robot_human1_animation.gif")

else
    println("Failed to converge. Aborting...")
end