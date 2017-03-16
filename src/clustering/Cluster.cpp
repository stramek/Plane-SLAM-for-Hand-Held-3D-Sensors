//

#include "include/clustering/Cluster.h"


// Created by stramek on 11.03.17.

Cluster::Cluster() {}

Cluster::Cluster(const Point3f &point) : point(point) {}

Cluster::~Cluster() {

}

const Point3f &Cluster::getPoint() const {
    return point;
}

void Cluster::setPoint(const Point3f &point) {
    Cluster::point = point;
}

const Cluster * Cluster::getLCluster() const {
    return clusters.at(0);
}

const Cluster * Cluster::getRCluster() const {
    return clusters.at(1);
}

const bool Cluster::hasCluster() const {
    return clusters.size() > 1;
}

const array<Cluster *, 2> &Cluster::getClusters() const {
    return clusters;
}

void Cluster::setLCluster(Cluster *cluster) {
    clusters.at(0) = cluster;
}

void Cluster::setRCluster(Cluster *cluster) {
    clusters.at(1) = cluster;
}

Cluster * Cluster::merge(Cluster *clusterToMerge) {
    Cluster* cluster = new Cluster();
    cluster->setLCluster(this);
    cluster->setRCluster(clusterToMerge);
    return cluster;
}



