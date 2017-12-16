/*
 * utils.cpp
 *
 *  Created on: Nov 4, 2017
 *      Author: dumbledore
 */

#include "utils.hpp"
#include <cmath>
#include <random>
#include <iostream>
#include "helper_functions.h"
#include <assert.h>

Utils::Utils() {
  // nothing for now
}

Utils::~Utils() {
  // nothing for now
}


double Utils::getTotalWeight(std::vector<Particle>& particles)
{
  // normalize probabilities
  double totalWeight = 0.0;

  // compute total weight
  for (Particle thisparticle : particles)
  {
    totalWeight = totalWeight + thisparticle.weight;
  }

  return (totalWeight);

}//void Utils::getTotalWeight



void Utils::normalizeProbabilities(std::vector<Particle>& particles)
{
  // normalize probabilities
  double totalWeight = 0.0;

  // compute total weight
  for (Particle thisparticle : particles)
  {
    totalWeight = totalWeight + thisparticle.weight;
  }

  /*assert(totalWeight > std::numeric_limits<double>::epsilon());*/
  if(totalWeight == 0)
  {
    for (Particle& thisparticle : particles)
        {
          thisparticle.weight = 0;
        }
  }else
  {
    // now normalize
      for (Particle& thisparticle : particles)
        {
          thisparticle.weight = thisparticle.weight/totalWeight;
        }
  }




}//void Utils::normalizeProbabilities

double Utils::computeMatchProbability(const MappedLandmarks& analyzedObservations, const std::vector<double>& std_landmark)
{
  double denominator = 2 * M_PI * std_landmark[0] * std_landmark[1];
  double exponential_xTerm = std::pow(analyzedObservations.observationInMapCoordinates.x - analyzedObservations.nearestMatchingLandmarkInMapCoordinates.x, 2.0)/(2 * std_landmark[0] * std_landmark[0] );
  double exponential_yTerm = std::pow(analyzedObservations.observationInMapCoordinates.y - analyzedObservations.nearestMatchingLandmarkInMapCoordinates.y, 2.0)/(2 * std_landmark[1] * std_landmark[1] );

  return(std::exp(-1*(exponential_xTerm + exponential_yTerm))/denominator);
}

Map::single_landmark_s Utils::transformFromVehicleToMapCoordinates(const Particle& particle, const LandmarkObs& thisObservation)
{
  Map::single_landmark_s translatedCoordinate;
  translatedCoordinate.x = static_cast<float>(particle.x + (std::cos(particle.theta) * thisObservation.x) - (std::sin(particle.theta) * thisObservation.y));
  translatedCoordinate.y = static_cast<float>(particle.y + (std::sin(particle.theta) * thisObservation.x) + (std::cos(particle.theta) * thisObservation.y));

  return (translatedCoordinate);
}

double Utils::euclideanDistance(const Particle& thisobject, const Map::single_landmark_s& thisObservation)
{

  return(std::sqrt((std::pow(thisobject.x - thisObservation.x, 2.0)) + std::pow(thisobject.y - thisObservation.y, 2.0)));

}


double Utils::euclideanDistance(const Map::single_landmark_s& thisobject, const Map::single_landmark_s& thisObservation)
{

  return(std::sqrt((std::pow(thisobject.x - thisObservation.x, 2.0)) + std::pow(thisobject.y - thisObservation.y, 2.0)));

}

double Utils::NormalizeYawAngle(const double angle) {

  if (std::abs(angle) / (M_PI) > 1) {
    //https://stackoverflow.com/questions/24234609/standard-way-to-normalize-an-angle-to-%CF%80-radians-in-java
    return (std::atan2(std::sin(angle), std::cos(angle)));

  } else {
    return (angle);
  }

}/* Utils::NormalizeYawAngle */

void Utils::doNoiseFreeParticlePrediction(
    double dt, const double velocity, double yawAngle,
    const double yaw_rate, std::array<double, 3>& NoiseFreePrediction) {

  if (std::fabs(yaw_rate) > 0.001) {
    // yaw rate is non-zero

    // predict x coordinate in map frame
    NoiseFreePrediction[0] = ((velocity / yaw_rate) * (std::sin(yawAngle + yaw_rate * dt) - std::sin(yawAngle)));

    // predict y coordinate in map frame
    NoiseFreePrediction[1] = ((velocity / yaw_rate) * (-std::cos(yawAngle + yaw_rate * dt) + std::cos(yawAngle)));

    // predict new yaw in map frame
    NoiseFreePrediction[2] = (yaw_rate * dt);

  } else {

    NoiseFreePrediction[0] = ((velocity) * std::cos(yawAngle) * dt);

    NoiseFreePrediction[1] = ((velocity) * std::sin(yawAngle) * dt);

    NoiseFreePrediction[2] = (yaw_rate * dt);

  }

}

void Utils::generateNoiseComponents(const std::vector<double>& sigma_pos, std::default_random_engine& randomEngine, std::normal_distribution<double>& dist_x, std::normal_distribution<double> dist_y, std::normal_distribution<double>& dist_theta,
                                    std::array<double, 3>& noiseComponent) {

  /*std::random_device rd;
  std::default_random_engine randomEngine( rd() );*/
  /*std::default_random_engine randomEngine;

  // Create a normal (Gaussian) distribution for x centered around initialNoisy_X
  std::normal_distribution<double> dist_x(0.0, sigma_pos[0]);

  // Create a normal (Gaussian) distribution for y centered around initialNoisy_Y
  std::normal_distribution<double> dist_y(0.0, sigma_pos[1]);

  // Create a normal (Gaussian) distribution for theta centered around initialNoisy_theta
  std::normal_distribution<double> dist_theta(0.0, sigma_pos[2]);*/

  // add noise to x-coordinate
  noiseComponent[0] = dist_x(randomEngine);

  // add noise to y-coordinate
  noiseComponent[1] = dist_y(randomEngine);

  // add noise to yaw-angle
  noiseComponent[2] = dist_theta(randomEngine);

}
