/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <array>
#include <limits>
#include "particle_filter.h"
#include "utils.hpp"
#include <assert.h>
#include "mappedLandmarks.hpp"
#include <map>
using namespace std;

ParticleFilter::ParticleFilter()
{
  // set the flag
  is_initialized = false;

  // set the number of particles
  num_particles = 51;

  // reserve the capacity to hold particles
  particles.reserve(num_particles);

  // reserve the capacity to hold new particles which will then be swapped after resampling.
  newParticles.reserve(num_particles);

  // occupy space.
  for (unsigned particleNumber = 0; particleNumber < num_particles;
      particleNumber++) {
    Particle newParticle;
    newParticle.id = particleNumber;
    particles.push_back(newParticle);
    newParticles.push_back(newParticle);

  }


}//ParticleFilter::ParticleFilter()

void
ParticleFilter::init(const double initialNoisy_X, const double initialNoisy_Y,
    const double initialNoisy_theta, const std::vector<double>& sigma_pos)
{

  // initial x, y, and theta estimate of the vehicle are either guesses, or from a noisy sensor, e.g., GPS.
  // Create a normal (Gaussian) distribution for x centered around initialNoisy_X
  std::normal_distribution<double> dist_x(initialNoisy_X, sigma_pos[0]);

  // Create a normal (Gaussian) distribution for y centered around initialNoisy_Y
  std::normal_distribution<double> dist_y(initialNoisy_Y, sigma_pos[1]);

  // Create a normal (Gaussian) distribution for theta centered around initialNoisy_theta
  std::normal_distribution<double> dist_theta(initialNoisy_theta,
      sigma_pos[2]);


  /*initialize each particle in particles vector -- the assigned positions include noise.
   The noise is characterized by a gaussian distribution centered around initialNoisy_ (e.g., GPS) coordinates
   with standard-deviation provided as an input.*/
  for (Particle &particle : particles)
  {
    particle.x = dist_x(randomEngine);
    particle.y = dist_y(randomEngine);
    particle.theta = dist_theta(randomEngine);
    particle.weight = 1.0;
  }

  // initialization is complete.
  is_initialized = true;

} //void ParticleFilter::init

void
ParticleFilter::prediction(double delta_t, const std::vector<double>& sigma_pos,
    double velocity, double yaw_rate)
{
  // prediction is performed in the map coordinates
  // prediction will be computed in two stages: determinisitic (i.e., noise-free part) and non-deterministic (i.e., noisy) component.

  // Create a normal (Gaussian) distribution for x centered around initialNoisy_X
  std::normal_distribution<double> dist_x(0.0, sigma_pos[0]);

  // Create a normal (Gaussian) distribution for y centered around initialNoisy_Y
  std::normal_distribution<double> dist_y(0.0, sigma_pos[1]);

  // Create a normal (Gaussian) distribution for theta centered around initialNoisy_theta
  std::normal_distribution<double> dist_theta(0.0, sigma_pos[2]);

  // for each particle
  for (Particle &particle : particles)
  {

    // do the noise-free prediction
    utils.doNoiseFreeParticlePrediction(delta_t, velocity, particle.theta, yaw_rate, noiseFreePredictionComponent);

    // compute the noise component
    utils.generateNoiseComponents(sigma_pos, randomEngine, dist_x, dist_y, dist_theta, noiseComponent);

    // compute the new particle state
    particle.x = particle.x + noiseFreePredictionComponent[0] + noiseComponent[0];
    particle.y = particle.y + noiseFreePredictionComponent[1] + noiseComponent[1];
    particle.theta = particle.theta + noiseFreePredictionComponent[2] + noiseComponent[2];

  }

}	//ParticleFilter::prediction

void
ParticleFilter::associateSensedLandmarkstoMapLandmarks(
    Particle particle, const double sensor_range,
    std::vector<LandmarkObs> observations, const Map& myMap, std::vector<MappedLandmarks>& analyzedObservations)
{

  double currentDistance;
  Map::single_landmark_s observationInMapCoordinates;
  LandmarkObs thisObservation;
  size_t i;

  for (i= 0; i < observations.size(); i++)
  {
    thisObservation = observations[i];
    thisObservation.id = -1; // no closest landmark within radar range found.
    analyzedObservations[i].id = -1;
    /*
     * Each particle represents the hypothetical position of the car after the prediction step.
     * Each hypothetical car represents our guess on the actual (i.e., accurate) position of the car at the next time step.
     * Furthermore, it is assumed that the measurements are taken from the sensors in the hypothetical car,
     * and thus, the measurements are assumed to be in the coordinate system located at the hypothetical car (i.e., the particle)
     *
     * As a result, each observation (i.e., measurement of landmarks captured by the sensors assumed to be located in the hypothetical car)
     * must be translated into map coordinates in order to compare it against the actual position of the landmarks as given in the map.
     *
     * Once each observation has been translated from the frame of reference of the hypothetical car (i.e., the particle),
     * it (i.e., observation post-translation into map coordinates) is paired with the closest actual landmark on the map.
     * The pairing is required since the measurements usually have errors, and the sensed location of the landmark will
     * most probably will not be exactly at the coordinates shown on the map. In other words, the difference between sensed
     * location of the landmark and the actual (aka ground truth) represents the noise in the measurements.
     *
     * Once the pairing is complete, the probability that a location as sensed by the sensors is associated to the paired (i.e., the closest actual) landmark
     * is computed using gaussian distribution, centered around the mean location of the actual "ground-truth" landmark, and standard-deviation of the sensors.
     * Practically, only the distance between the sensed location and the standard-deviations are required.
     *
     * Thereafter, the confidence that the hypothetical car (i.e., the particle) represents the actual position of the vehicle is deduced by
     * computing the total probability that _all_ observations correspond to actual "ground truth" landmarks with high confidence.
     * The probability that a given sensed landmark corresponds to an actual (closest matched) landmark is independent of other observations,
     * and hence probabilities are multiplied together.
     *
     * Thereafter, the particle with the high total probability (over all observations at the given time step) represents the best estimate of the actual
     * position of the vehicle.
     *
     */

    // translate the observation from the particle's frame of reference to the map reference.
    observationInMapCoordinates = utils.transformFromVehicleToMapCoordinates(particle, thisObservation);

    double distance = std::numeric_limits<double>::max();
    for (const Map::single_landmark_s &thisLandmarkFromMap : myMap.landmark_list) {

      // proceed only if the landmark from the map is within the sensor_range
      if (utils.euclideanDistance(particle, thisLandmarkFromMap) <= sensor_range)
      {
        currentDistance = utils.euclideanDistance(observationInMapCoordinates, thisLandmarkFromMap);

        // find the actual landmark on the map which is closest to the sensed landmark.
        if(currentDistance < distance){
          distance = currentDistance;
          // later on, we can access the details of the landmark from the map with id = n as map.landmark_list[n-1].id, map.landmark_list[n-1].x, map.landmark_list[n-1].y
          thisObservation.id = thisLandmarkFromMap.id;
          analyzedObservations[i].id = thisLandmarkFromMap.id;
          analyzedObservations[i].observationInMapCoordinates = observationInMapCoordinates;
          analyzedObservations[i].nearestMatchingLandmarkInMapCoordinates = thisLandmarkFromMap;
          analyzedObservations[i].euclideanDistance = currentDistance; // distance between the sensed location of the landmark (post translation) and the closest landmark on the map.

        }
        else
        {

        }

      }//if

    }//for (const Map::single_landmark_s &thisLandmarkFromMap : map.landmark_list)

  }//for (LandmarkObs &thisObservation : observations)

}//void ParticleFilter::associateSensedLandmarkstoMapLandmarks



void
ParticleFilter::updateWeights(double sensor_range, const std::vector<double>& std_landmark,
    const std::vector<LandmarkObs> &observations, const Map &map_landmarks)
{
  /*
   * For each particle at the next time-step, include only those map-landmarks which are within the range of the sensors
   * observations: sensed-landmarks by the sensors in the vehicle
   * map_landmarks: all landmarks in the map
   * */

  size_t i;
  for (Particle &particle : particles)
  {
    // the location of the particle will determine the subset of landmarks from the map to be considered for mapping.


    std::vector<MappedLandmarks> analyzedObservations;
    analyzedObservations.reserve(observations.size());

    // For each observed landmark (aka measurement) find the closest real (aka on the map) landmark
    this->associateSensedLandmarkstoMapLandmarks(particle, sensor_range,
        observations, map_landmarks, analyzedObservations);

    // Compute probability that each observed landmark translated to map coordinates corresponds to the associated landmark measurement.
    double probabilityofMatch = 1.0;
    for (i=0; i<observations.size(); i++)
    {
      // the total probability of measuring the "scene" -- indicator of the position of the car.
      probabilityofMatch = probabilityofMatch * utils.computeMatchProbability(analyzedObservations[i], std_landmark);

    }

    particle.weight = probabilityofMatch;

  }

  utils.normalizeProbabilities(particles);

}

void
ParticleFilter::resample()
{

  std::random_device rd;
  std::default_random_engine randomEngine( rd() );


  double totalWeight = utils.getTotalWeight(particles);
  if((totalWeight < 1 - this->num_particles*std::numeric_limits<double>::epsilon()) ||  (totalWeight > 1 + this->num_particles*std::numeric_limits<double>::epsilon()))
  {
    //std::cout << "Total Probability: " << utils.getTotalWeight(particles) << std::endl;
    assert((totalWeight < 1 - this->num_particles*std::numeric_limits<double>::epsilon()) ||  (totalWeight > 1 + this->num_particles*std::numeric_limits<double>::epsilon()));
  }


  // copy the weights into a separate array
  std::vector<double> weights;
  weights.reserve(this->num_particles);

  // create an array containing the weights of each particle
  for (size_t i = 0; i < particles.size(); i++)
  {
    //weights[i] = particles[i].weight;
    weights.push_back(particles[i].weight);
  }

  // the distribution is given by the weights of each particle.
  std::discrete_distribution<>d(weights.begin(), weights.end());

  std::map<int, int> newSample;
  for(size_t i = 0; i< this->num_particles; i++)
  {
    /* probabilistically pick a particle in proportion to its weight.
     * that is, a particle with a higher weight will be picked more often (selection with replacement).
    */
    ++newSample[d(randomEngine)];
  }

  int i = 0;
  for(auto p : newSample)
  {
    for (int j = 0; j< p.second; j++)
    {
      newParticles[i++] = particles[p.first];
    }
  }

  particles = newParticles;

}

Particle
ParticleFilter::SetAssociations(Particle particle,
    std::vector<int> associations, std::vector<double> sense_x,
    std::vector<double> sense_y)
{
  //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates

  //Clear the previous associations
  particle.associations.clear();
  particle.sense_x.clear();
  particle.sense_y.clear();

  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;

  return particle;
}

string
ParticleFilter::getAssociations(Particle best)
{
  vector<int> v = best.associations;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}
string
ParticleFilter::getSenseX(Particle best)
{
  vector<double> v = best.sense_x;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}
string
ParticleFilter::getSenseY(Particle best)
{
  vector<double> v = best.sense_y;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}
