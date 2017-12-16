/*
 * mappedLandmarks.hpp
 *
 *  Created on: Nov 6, 2017
 *      Author: dumbledore
 */

#ifndef SRC_MAPPEDLANDMARKS_HPP_
#define SRC_MAPPEDLANDMARKS_HPP_

#include "map.h"

class MappedLandmarks {
 public:

  int id;
  Map::single_landmark_s observationInMapCoordinates;
  Map::single_landmark_s nearestMatchingLandmarkInMapCoordinates;
  double euclideanDistance;


 private:


};



#endif /* SRC_MAPPEDLANDMARKS_HPP_ */
