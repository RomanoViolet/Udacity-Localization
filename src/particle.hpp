/*
 * particle.hpp
 *
 *  Created on: Nov 4, 2017
 *      Author: dumbledore
 */

#ifndef SRC_PARTICLE_HPP_
#define SRC_PARTICLE_HPP_

#include <vector>

class Particle {
 public:

  unsigned id;
  double x;
  double y;
  double theta;
  double weight;
  std::vector<int> associations;
  std::vector<double> sense_x;
  std::vector<double> sense_y;

 private:


};

#endif /* SRC_PARTICLE_HPP_ */
