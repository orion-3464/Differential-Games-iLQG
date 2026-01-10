using LinearAlgebra

d_prox = 1.5
d_th = 3.0
indicator(condition) = condition ? 1.0 : 0.0


function robot_cost(g, x, u, t)
    p_robot_goal = SVector(0.0, 0.0)
    R = Diagonal([0.1, 0.1])

    px, py, θ, v= x[1:4]

    p_r = SVector(px, py)
    p_h1 = SVector(x[5], x[6])
    p_h2 = SVector(x[9], x[10])

    dist_r_h1 = norm(p_r-p_h1)
    dist_r_h2 = norm(p_r-p_h2)
    dist_goal = norm(p_r-p_robot_goal)

    cost_proximity1 = indicator(dist_r_h1<d_prox)*(d_prox-dist_r_h1)^2
    cost_proximity2 = indicator(dist_r_h2<d_prox)*(d_prox-dist_r_h2)^2
    cost_goal = dist_goal^2
    cost_input = dot(u[1:2], R*u[1:2])

    return cost_proximity1 + cost_proximity2 + 0.5*cost_goal + 5*cost_input
end

function human1_cost(g, x, u, t)
    p_human1_goal = SVector(3.0, -3.0)
    R = Diagonal([0.1, 0.1])

    px, py, θ, v= x[5:8]

    p_r = SVector(x[1], x[2])
    p_h1 = SVector(px, py)
    p_h2 = SVector(x[9], x[10])

    dist_r_h1 = norm(p_r-p_h1)
    dist_h1_h2 = norm(p_h1-p_h2)
    dist_goal = norm(p_h1-p_human1_goal)

    cost_proximity1 = indicator(dist_r_h1<d_prox)*(d_prox-dist_r_h1)^2
    cost_proximity2 = indicator(dist_h1_h2<d_prox)*(d_prox-dist_h1_h2)^2
    cost_goal = dist_goal^2
    cost_input = dot(u[3:4], R*u[3:4])


    return cost_proximity1 + cost_proximity2 + 0.5*cost_goal + 5*cost_input
end

function human2_cost(g, x, u, t)
    p_human2_goal = SVector(-3.0, 3.0)
    R = Diagonal([0.1, 0.1])

    px, py, θ, v= x[9:12]

    p_r = SVector(x[1], x[2])
    p_h1 = SVector(x[5], x[6])
    p_h2 = SVector(px, py)

    dist_r_h2 = norm(p_r-p_h2)
    dist_h1_h2 = norm(p_h1-p_h2)
    dist_goal = norm(p_h2-p_human2_goal)

    cost_proximity1 = indicator(dist_h1_h2<d_prox)*(d_prox-dist_h1_h2)^2
    cost_proximity2 = indicator(dist_r_h2<d_prox)*(d_prox-dist_r_h2)^2
    cost_goal = dist_goal^2
    cost_input = dot(u[5:6], R*u[5:6])

    return cost_proximity1 + cost_proximity2 + 0.5*cost_goal + 5*cost_input
end