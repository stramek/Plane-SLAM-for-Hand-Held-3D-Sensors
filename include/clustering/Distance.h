//
// Created by stramek on 11.03.17.
//

#ifndef PROJEKTMAGISTERSKI_CLUSTERING_DISTANCE_H
#define PROJEKTMAGISTERSKI_CLUSTERING_DISTANCE_H


class Distance {
public:
    Distance();

    Distance(double distance);

    double getDistance() const;

    void setDistance(double distance);

private:
    double distance;

};


#endif //PROJEKTMAGISTERSKI_CLUSTER_H
