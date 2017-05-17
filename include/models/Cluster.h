//
// Created by mordimer on 11.03.17.
//

#ifndef PROJEKTMAGISTERSKI_CLASTER_H
#define PROJEKTMAGISTERSKI_CLASTER_H

#include "set"

using namespace std;

class Cluster {
private:
    std::pair<int , int> linkedIndexes;
    float distanceBetweenLinks;
public:
    void setLinkedIndexes(const pair<int, int> &linkedIndexes);

    void setDistanceBetweenLinks(float distanceBetweenLinks);

    bool operator<(const Cluster &cluster) const{
        return this->distanceBetweenLinks > cluster.getDistanceBetweenLinks();
    }

    const pair<int, int> &getLinkedIndexes() const;

    float getDistanceBetweenLinks() const;
};

#endif //PROJEKTMAGISTERSKI_CLASTER_H
