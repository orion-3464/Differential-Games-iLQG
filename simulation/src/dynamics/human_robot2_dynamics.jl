using iLQGames
import iLQGames: dx, xyindex
using StaticArrays, Random

nx, nu, dt, game_horizon = 12, 6, 0.1, 400

rng = MersenneTwister(12345)

const num_surprises = 3
const surprise_times = sort(randperm(rng, 50)[1:num_surprises])
const surprise_agents = rand(rng, 1:3, num_surprises)  # 1: TurtleBot, 2: Human1, 3: Human2
const surprise_magnitudes = 8.0 .+ 8.0 .* rand(rng, num_surprises)  # 5-10

println("Τυχαία απρόβλεπτα γεγονότα:")
for i in 1:num_surprises
    agent_name = surprise_agents[i] == 1 ? "TurtleBot" : 
                (surprise_agents[i] == 2 ? "Human1" : "Human2")
    println("  Βήμα $(surprise_times[i]): $agent_name, μέγεθος $(round(surprise_magnitudes[i], digits=2))")
end

struct Game2 <: ControlSystem{dt, nx, nu} end

function dx(cs::Game2, x, u, t)
    turtle_acc = u[1]
    human1_acc = u[3]
    human2_acc = u[5]
    
    
    for i in 1:num_surprises
        τ = surprise_times[i] * dt
        σ = 0.2
        
        transition = exp(-((t - τ)/σ)^2)
        
        if surprise_agents[i] == 1  # TurtleBot
            turtle_acc += surprise_magnitudes[i] * transition
        elseif surprise_agents[i] == 2  # Human1
            human1_acc += surprise_magnitudes[i] * transition
        else  # Human2
            human2_acc += surprise_magnitudes[i] * transition
        end
    end
    
    return SVector{12}(
        x[3] * cos(x[4]),
        x[3] * sin(x[4]),
        turtle_acc, 
        u[2],
        x[7] * cos(x[8]),
        x[7] * sin(x[8]),
        human1_acc, 
        u[4],
        x[11] * cos(x[12]),
        x[11] * sin(x[12]),
        human2_acc, 
        u[6]
    )
end

xyindex(::Game2) = ((1,2), (5,6), (9, 10))