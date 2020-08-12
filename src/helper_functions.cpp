#include "helper_functions.h"

// Random Generators
std::random_device rd;
std::mt19937 gen(rd());


double helper::generateRandom() {
    std::uniform_real_distribution<double> distribution(0.0, 1.0);
    return distribution(gen);
}


double helper::generateRandomGaussian(double mean, double variance) {
    std::normal_distribution<double> distribution(mean, variance);
    return distribution(gen);
}

double helper::probabilityDensity(double mu, double sigma, double x) {
    return exp(-(pow((mu - x), 2)) / (pow(sigma, 2)) / 2.0) / sqrt(2.0 * M_PI * (pow(sigma, 2)));
}

double helper::max(std::vector<double> vec) {
    double max = 0;
    for (auto v : vec) {
        if (v > max)
            max = v;
    }
    return max;
}

double helper::mod(double first_term, double second_term) {
    return first_term - (second_term)*floor(first_term/(second_term));
}


