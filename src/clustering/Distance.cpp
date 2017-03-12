//
// Created by stramek on 11.03.17.
//

#include "include/clustering/Distance.h"

Distance::Distance() {}

Distance::Distance(double distance) : distance(distance) {}

double Distance::getDistance() const {
    return distance;
}

void Distance::setDistance(double distance) {
    Distance::distance = distance;
}
