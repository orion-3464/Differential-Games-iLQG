using LinearAlgebra

dt = 0.1
T = 5.0  

# Cost functions' parameters
d_prox = 1.0
t_goal = 2.0
d_lane = 1.5

# Car1 (Compact) parameters 
p_goal_c1 = SVector(2.0, 2.0)
v_ref_c1 = 7.0
v_max_c1 = 10.0
v_min_c1 = -5.0 
R_c1 = Diagonal([0.2, 0.1])

# Car2 (SUV) parameters 
p_goal_c2 = SVector(0.0, 2.5)
v_ref_c2 = 8.0
v_max_c2 = 10.0
v_min_c2 = -5.0 
R_c2 = Diagonal([0.3, 0.15])

# Utils
indicator(condition) = condition ? 1.0 : 0.0

function goal_cost(p, p_goal, t, T)
    if t > T - t_goal
        dx = p[1] - p_goal[1]
        dy = p[2] - p_goal[2]
        return dx^2 + dy^2
    else
        return 0.0
    end
end

# Players' costs
function car1_cost(g, x, u, t)
    px, py, θ, v, φ = x[1:5]

    # Proximity cost
    p_c1 = SVector(px, py)
    p_c2 = SVector(x[6], x[7])
    dist = norm(p_c1-p_c2)
    cost_proximity = indicator(dist<d_prox)*(d_prox-dist)^2

    cost_goal = goal_cost(p_c1, p_goal_c1, t*dt, T)

    cost_input = dot(u[1:2], R_c1*u[1:2])

    dl = abs(py - 2.0)
    cost_lane_center = dl^2

    cost_lane_boundary = indicator(dl>d_lane)*(d_lane-dl)^2

    const_nominal_speed = (v-v_ref_c1)^2

    cost_speed_bounds = indicator(v > v_max_c1)*(v-v_max_c1)^2 + 
                        indicator(v < v_min_c1)*(v_min_c1-v)^2

    return 5.0*cost_proximity + 15.0*cost_goal + 1.0*cost_input +
           5.0*cost_lane_center + 15.0*cost_lane_boundary + 5.0*const_nominal_speed +
           10.0*cost_speed_bounds
end

function car2_cost(g, x, u, t)
    px, py, θ, v, φ = x[6:10]

    # Proximity cost
    p_c2 = SVector(px, py)
    p_c1 = SVector(x[1], x[2])
    dist = norm(p_c1-p_c2)
    cost_proximity = indicator(dist<d_prox)*(d_prox-dist)^2

    cost_goal = goal_cost(p_c2, p_goal_c2, t*dt, T)

    cost_input = dot(u[3:4], R_c2*u[3:4])

    dl = abs(px - 0.0)
    cost_lane_center = dl^2

    cost_lane_boundary = indicator(dl>d_lane)*(d_lane-dl)^2

    const_nominal_speed = (v-v_ref_c2)^2

    cost_speed_bounds = indicator(v > v_max_c2)*(v-v_max_c2)^2 + 
                        indicator(v < v_min_c2)*(v_min_c2-v)^2

    return 5.0*cost_proximity + 15.0*cost_goal + 1.0*cost_input +
           5.0*cost_lane_center + 15.0*cost_lane_boundary + 5.0*const_nominal_speed +
           10.0*cost_speed_bounds
end