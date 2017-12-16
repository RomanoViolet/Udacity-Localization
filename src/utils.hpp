/*
 * utils.hpp
 *
 *  Created on: Nov 4, 2017
 *      Author: dumbledore
 */

#ifndef SRC_UTILS_HPP_
#define SRC_UTILS_HPP_

#include <array>
#include "particle.hpp"
#include <algorithm>
#include <functional>
#include "helper_functions.h"
#include "map.h"
#include "mappedLandmarks.hpp"


class Utils {
 public:
  // default constructor
  Utils();

  // default destructor
  ~Utils();

  // helper function to perform prediction of the particle at the next time step ignoring noise.
  void doNoiseFreeParticlePrediction(
      const double delta_t, const double velocity, double yawAngle,
      const double yaw_rate, std::array<double, 3>& NoiseFreePrediction);


  void generateNoiseComponents(
      const std::vector<double>& sigma_pos, std::default_random_engine& randomEngine, std::normal_distribution<double>& dist_x, std::normal_distribution<double> dist_y, std::normal_distribution<double>& dist_theta,
      std::array<double, 3>& noiseComponent);


  double NormalizeYawAngle(const double angle);

  /*template <class T>
  inline double euclideanDistance(const T& thisobject, const Map::single_landmark_s& thisObservation);*/

  //this should ideally be a templated function. Oh my giddy aunt.
  double euclideanDistance(const Particle& thisobject, const Map::single_landmark_s& thisObservation);
  double euclideanDistance(const Map::single_landmark_s& thisobject, const Map::single_landmark_s& thisObservation);

  Map::single_landmark_s transformFromVehicleToMapCoordinates(const Particle& particle, const LandmarkObs& thisObservation);

  double computeMatchProbability(const MappedLandmarks& analyzedObservations, const std::vector<double>& std_landmark);

  void normalizeProbabilities(std::vector<Particle>& particles);

  double getTotalWeight(std::vector<Particle>& particles);


 private:

};


/*#include "templatedUtils.cpp"*/
#endif /* SRC_UTILS_HPP_ */
