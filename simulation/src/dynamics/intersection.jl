using iLQGames
import iLQGames: dx, xyindex

nx, nu, dt, game_horizon = 10, 4, 0.1, 500
L1, L2 = 2.5, 2.9 # wheelbases

struct Intersection <: ControlSystem{dt, nx, nu} end

dx(cs::Intersection, x, u, t) = SVector(
    # Car1 (Compact) dynamics - Bicycle kinematic model
    # [px, py, θ, v, φ]
    x[4]cos(x[3]),
    x[4]sin(x[3]),
    x[4]*tan(x[5])/L1,
    u[2],
    u[1],
    # Car2 (SUV) dynamics - Bicycle kinematic model
    # [px, py, θ, v, φ]
    x[9]cos(x[8]),
    x[9]sin(x[8]),
    x[9]*tan(x[10])/L2,
    u[4],
    u[3]
)

# Specify players' positions
xyindex(::Intersection) = ((1,2), (6,7))