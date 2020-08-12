#include "robot.h"


Robot::Robot(const World& world)
: _forward_noise(0.0001)
, _turn_noise(0.0001)
, _sensor_noise(0.0001)
, _world(world)
, _limit (true)
{
    _x = helper::generateRandom() * _world.size.x; // robot's x coordinate
    _y = helper::generateRandom() * _world.size.y; // robot's y coordinate
    _orientation = helper::generateRandom() * 2.0 * M_PI; // robot's orientation
}

Robot::~Robot(){}

void Robot::setPose(const double x, const double y, const double orientation) {

    if (_limit) {
        if (x < 0 || x >= _world.size.x)
            throw std::invalid_argument("X coordinate out of bound");
        if (y < 0 || y >= _world.size.y)
            throw std::invalid_argument("Y coordinate out of bound");
        if (orientation < 0 || orientation >= 2 * M_PI)
            throw std::invalid_argument("Orientation must be in [0..2pi]");
    }
    _x = x;
    _y = y;
    _orientation = orientation;
}

void Robot::setNoise(double forward_noise, double turn_noise, double sensor_noise) {
    _forward_noise = forward_noise;
    _turn_noise = turn_noise;
    _sensor_noise = sensor_noise;
}

double Robot::getX() const {
    return _x;
}

double Robot::getY() const {
    return _y;
}

double Robot::getOrientation() const {
    return _orientation;
}

Robot Robot::move(double turn, double forward) {
    if (forward < 0)
        throw std::invalid_argument("Robot cannot move backwards | forward should be a positive value");
    
    _orientation = _orientation + turn + helper::generateRandomGaussian(0.0, _turn_noise);
    _orientation = helper::mod(_orientation, 2 * M_PI);

    double dist = forward + helper::generateRandomGaussian(0.0, _forward_noise);
    _x = _x + (cos(_orientation) * dist);
    _y = _y + (sin(_orientation) * dist);

    if (_limit) {
        // cyclic truncate
        _x = helper::mod(_x, _world.size.x);
        _y = helper::mod(_y, _world.size.y);
    }

    Robot ret(_world);
    ret.setPose(_x, _y, _orientation);
    ret.setNoise(_forward_noise, _turn_noise, _sensor_noise);

    return ret;
}

void Robot::displayPose() const {
    std::cout<<"[Robot Pose] X:"<<_x<<"  Y:"<<_y<<"   Orientaion:"<<_orientation<<std::endl;
}

std::vector<double> Robot::sense() {
    
    std::vector<double> measurements(_world.landmarks.size());
    double dist;
    for (size_t i = 0; i < _world.landmarks.size(); ++i) {
        dist = sqrt(pow((_x - _world.landmarks[i].first), 2) 
                + pow((_y - _world.landmarks[i].second), 2));
        dist += helper::generateRandomGaussian(0.0, _sensor_noise);
        measurements[i] = dist;
    }

    return measurements;

}

std::string Robot::readSensors() {
    
    std::vector<double> measurements =  sense();
    std::string readings = "[";
    for (auto z : measurements) {
        readings += std::to_string(z) + " ";
    }
    readings += "]";

    return readings;
}


double Robot::getMeasurementProbability(const std::vector<double>& measurements) {
    double prob = 1.0;
    double dist;
    for (size_t i = 0; i < _world.landmarks.size(); ++i) {
        dist = sqrt(pow((_x-_world.landmarks[i].first), 2) + pow((_y - _world.landmarks[i].second), 2));
        prob *= helper::probabilityDensity(dist, _sensor_noise, measurements[i]);
    }

    return prob;
}
