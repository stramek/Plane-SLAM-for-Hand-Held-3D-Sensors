//
// Created by mordimer on 11.03.17.
//

#include "include/dataset/Cluster.h"

void Cluster::setFirstLinkIndex(int firstLinkIndex) {
    Cluster::firstLinkIndex = firstLinkIndex;
}

void Cluster::setSecondLinkIndex(int secondLinkIndex) {
    Cluster::secondLinkIndex = secondLinkIndex;
}

int Cluster::getFirstLinkIndex() const {
    return firstLinkIndex;
}

int Cluster::getSecondLinkIndex() const {
    return secondLinkIndex;
}

float Cluster::getDistanceBetweenLinks() const {
    return distanceBetweenLinks;
}

void Cluster::setDistanceBetweenLinks(float distanceBetweenLinks) {
    Cluster::distanceBetweenLinks = distanceBetweenLinks;
}


