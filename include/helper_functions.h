#pragma once

#include <iostream>
#include <random> 
#include <math.h>
#include <vector>



namespace helper {
/**
 * The function generates a random number between 
 * 0 and 1 
*/
double generateRandom();

/**
 * The function generates a random Gaussian
 * @param mean 
 * @param variance
*/
double generateRandomGaussian(double mean, double variance);

/**
 * The function returns a Gaussian Value
 * @param mu - mean 
 * @param sigma - standard deviation
 * @param x
*/
double probabilityDensity(double mu, double sigma, double x);

/**
 * The function returns a maximum value of a vector
 * @param vec
*/
double max(std::vector<double> vec);

/**
 * The function returns a modulus 
 * @param first_term
 * @param second_term
*/
double mod(double first_term, double second_term);

} //helper namespace

