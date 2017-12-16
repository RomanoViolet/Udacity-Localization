/*
 * templatedUtils.cpp
 *
 *  Created on: Nov 5, 2017
 *      Author: dumbledore
 */

#include "utils.cpp"

/*class Utils;*/

/*template class Utils::euclideanDistance<Particle>;
template class Utils::euclideanDistance<LandmarkObs>;*/

template class Utils::euclideanDistance<Particle>(const Particle& thisobject, const Map::single_landmark_s& thisObservation);
template class Utils::euclideanDistance<LandmarkObs>(const LandmarkObs& thisobject, const Map::single_landmark_s& thisObservation);



