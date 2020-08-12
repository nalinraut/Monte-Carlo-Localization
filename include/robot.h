#pragma once

#include <iostream>
#include <string>
#include <math.h>
#include <stdexcept>
#include <random>
#include <vector>
#include <memory>

#include <helper_functions.h>

/*
    World Size Type
*/
struct WorldSize {
    size_t x;
    size_t y;
}; // World Size


/*
    World structure
*/
struct World {
    World(const WorldSize& size, 
        const std::vector<std::pair<double, double>>& landmarks)
    : size(size)
    , landmarks(landmarks)
    {}
    WorldSize size;
    std::vector<std::pair<double, double>> landmarks;    
}; // World


class Robot{

public:
    /**
    * Constructs an object robot of this class
    * @param world Initialize the robot with world 
    */
    Robot(const World& world);

    /**
    * Destructor 
    */
    ~Robot();

    /**
     * Sets the pose of the robot
     * @param x x coordinate of Pose
     * @param y y coordinate of Pose
     * @param orientation orientation   
    */
    void setPose(const double, const double, const double);

    /**
     * Sets the noise for the robot motion and measurement
     * @param _forward_noise 
     * @param _turn_noise
     * @param _sensor_noise  
    */
    void setNoise(const double, const double, const double);
    
    /**
     * Returns the X coordinate of robot
    */
    double getX() const;
    
    /**
     * Returns the Y coordinate of robot
    */
    double getY() const;

    /**
     * Returns the orientation of robot
    */
    double getOrientation() const;
    
    /**
     * Sets the noise for the robot motion and measurement
     * @param turn 
     * @param forward  
    */
    Robot move(const double, const double);
    
    /**
     * Displays the pose of robot
    */
    void displayPose() const;
    
    /**
     * Returns the measurements from worl landmarks
    */
    std::vector<double> sense();
    
    /**
     * Returns the string of measurements
    */
    std::string readSensors();
    
    /**
     * Returns the measurement probability
     * @param measurements
    */
    double getMeasurementProbability(const std::vector<double>&);

private:
    double _x;
    double _y;
    double _orientation;
    double _forward_noise;
    double _turn_noise;
    double _sensor_noise;
    World _world;
    bool _limit = false;
}; // class Robot

