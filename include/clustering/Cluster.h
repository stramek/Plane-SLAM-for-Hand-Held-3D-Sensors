//
// Created by stramek on 11.03.17.
//

#ifndef PROJEKTMAGISTERSKI_CLUSTER_H
#define PROJEKTMAGISTERSKI_CLUSTER_H

#include "Distance.h"
#include "ClusterPair.h"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class Cluster {
private:
    Point3f point;
    array<Cluster*, 2> clusters;
public:
    Cluster();

    Cluster(const Point3f &point);

    virtual ~Cluster();

    const Point3f &getPoint() const;

    void setPoint(const Point3f &point);

    void setLCluster(Cluster* cluster);

    void setRCluster(Cluster* cluster);

    const Cluster* getLCluster() const;

    const Cluster* getRCluster() const;

    const bool hasCluster() const;

    const array<Cluster *, 2> &getClusters() const;

    Cluster * merge(Cluster* cluster);
};

#endif //PROJEKTMAGISTERSKI_CLUSTER_H