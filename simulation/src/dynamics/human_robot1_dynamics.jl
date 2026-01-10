using iLQGames
import iLQGames: dx, xyindex

nx, nu, dt, game_horizon = 12, 6, 0.1, 600

struct Game1 <: ControlSystem{dt, nx, nu} end

dx(cs::Game1, x, u, t) = SVector(
    # TurtleBot [px, py, v, θ]
    x[3]cos(x[4]),
    x[3]sin(x[4]),
    u[1],
    u[2],

    # Human1 [px, py, v, θ]
    x[7]cos(x[8]),
    x[7]sin(x[8]),
    u[3],
    u[4],

    # Human2 [px, py, v, θ]
    x[11]cos(x[12]),
    x[11]sin(x[12]),
    u[5],
    u[6],
)

xyindex(::Game1) = ((1,2), (5,6), (9, 10))