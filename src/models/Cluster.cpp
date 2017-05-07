//
// Created by mordimer on 11.03.17.
//

#include <vector>
#include <include/models/Cluster.h>
#include "include/models/Cluster.h"

void Cluster::setFirstLinkIndex(int firstLinkIndex) {
    Cluster::firstLinkIndex = firstLinkIndex;
    indexList.insert(firstLinkIndex);
}

void Cluster::setSecondLinkIndex(int secondLinkIndex) {
    Cluster::secondLinkIndex = secondLinkIndex;
    indexList.insert(secondLinkIndex);
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

void Cluster::mergeClildrenIndexes(Cluster &cluster1, Cluster &cluster2) {
    indexList.insert(cluster1.getIndexList().begin(), cluster1.getIndexList().end());
    indexList.insert(cluster2.getIndexList().begin(), cluster2.getIndexList().end());
}

std::set<int> &Cluster::getIndexList() {
    return indexList;
}

void Cluster::setIndexList(std::set<int> &indexList) {
    Cluster::indexList = indexList;
}



