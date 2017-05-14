//
// Created by mordimer on 11.03.17.
//

#include <vector>
#include <include/models/Cluster.h>
#include "include/models/Cluster.h"


void Cluster::setLinkedIndexes(const pair<int, int> &linkedIndexes) {
    Cluster::linkedIndexes = linkedIndexes;
}

void Cluster::setDistanceBetweenLinks(float distanceBetweenLinks) {
    Cluster::distanceBetweenLinks = distanceBetweenLinks;
}

const pair<int, int> &Cluster::getLinkedIndexes() const {
    return linkedIndexes;
}

float Cluster::getDistanceBetweenLinks() const {
    return distanceBetweenLinks;
}
