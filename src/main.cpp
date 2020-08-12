#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include "helper_functions.h"
#include "robot.h"


#include "matplotlibcpp.h" 
namespace plt = matplotlibcpp;



double evaluateRMSE(const Robot& entity, const std::vector<Robot>& entity_vec, const WorldSize& world_size) {
    double sum = 0.0;
    for (auto e : entity_vec) {
        double dx = helper::mod((e.getX() - entity.getX() + (world_size.x / 2.0)), world_size.x) - (world_size.x / 2.0);
        double dy = helper::mod((e.getY() - entity.getY() + (world_size.y / 2.0)), world_size.y) - (world_size.y / 2.0);
        double error = sqrt(pow(dx, 2) + pow(dy, 2));
        sum += error;
    }
    return sum/entity_vec.size();
}

void visualization(int n, Robot robot, int step, 
                    std::vector<Robot> particles, 
                    std::vector<Robot> resampled_particles,
                    std::vector<std::pair<double, double>> landmarks) {
    

    // Graph format
    plt::title("MCL, Step: "+std::to_string(step));
    plt::xlim(0, 100);
    plt::ylim(0, 100);
    
    // Draw particles in green 
    for (auto p : particles) {
        plt::plot({p.getX()}, {p.getY()}, "go");
    }
    
    // Draw resampled particles in yellow 
    for (auto r : resampled_particles) {
        plt::plot({r.getX()},{r.getY()},"yo");
    }
    

    // Draw landmarks in red
    for (auto l : landmarks) {
        plt::plot({l.first},{l.second}, "ro");
    }
    
    // Draw robot position in blue
    plt::plot({robot.getX()},{robot.getY()},"bo");
    plt::save("../assets/step"+std::to_string(step)+".png");
    plt::clf();

}

void mcl() {
    std::vector<std::pair<double, double>> landmarks = { { 20.0, 20.0 }, { 20.0, 80.0 }, { 20.0, 50.0 },
    { 50.0, 20.0 }, { 50.0, 80.0 }, { 80.0, 80.0 },
    { 80.0, 20.0 }, { 80.0, 50.0 } }; 
    WorldSize size{100, 100};
    World world(size, landmarks);
    Robot robot(world);

    // Create a set of particles
    int particles_n = 1000;
    std::vector<Robot> particles;

    // Set noise for all particles
    for(auto i = 0; i < particles_n; ++i) {
        Robot p(world);
        p.setNoise(0.05, 0.05, 5.0);
        particles.push_back(p);
    }

    // Initialize a measurement vector
    std::vector<double> measurements;

    // Iterating 50 times over the set of particles
    int steps = 50 ;
    for (int t = 0; t < steps; ++t) {

        // Move the robot and sense the environment afterwards
        robot = robot.move(0.1, 5.0);
        measurements = robot.sense();
        
        // Simulate a robot motion for each of these particles
        std::vector<Robot> plot_particles(particles_n, world);
        for (int i = 0; i < particles_n; ++i ) {
            plot_particles[i] = particles[i].move(0.1, 5.0);
        }

        std::vector<double> weights(particles_n);
        for (int i = 0; i < particles_n; ++i) {
            weights[i] = particles[i].getMeasurementProbability(measurements);
        }

        /* Resample the particles with a sample probability proportional to the importance
        weight */
        int index = helper::generateRandom() * particles_n;

        double beta = 0.0;
        double max_weight = helper::max(weights);
        
        std::vector<Robot> resampled_particles(particles_n, world);
        for (int i = 0; i < particles_n; ++i) {
            beta += helper::generateRandom() * 2.0 * max_weight;
            
            while (beta > weights[index]) {
                beta -= weights[index];
                index = helper::mod((index + 1), particles_n);
            }
            resampled_particles[i] = particles[index];
        }

        std::cout << "Step = " << t << ", Evaluation = " << evaluateRMSE(robot, particles, size) << std::endl;
        for (int k = 0; k < particles_n; k++) {
            particles[k] = resampled_particles[k];
        }
        
        visualization(particles_n, robot, t, plot_particles, resampled_particles, world.landmarks);
    }
}

int main() {
    mcl();    
    return 0; 
}