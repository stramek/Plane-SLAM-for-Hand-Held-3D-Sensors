//
// Created by mordimer on 11.03.17.
//

#ifndef PROJEKTMAGISTERSKI_CLASTER_H
#define PROJEKTMAGISTERSKI_CLASTER_H

#include "set"

using namespace std;

class Cluster {
private:
    int firstLinkIndex;
    int secondLinkIndex;
    float distanceBetweenLinks;
    set<int> indexList;
public:
    void setFirstLinkIndex(int firstLinkIndex);

    void setSecondLinkIndex(int secondLinkIndex);

    void setDistanceBetweenLinks(float distanceBetweenLinks);

    int getFirstLinkIndex() const;

    int getSecondLinkIndex() const;

    float getDistanceBetweenLinks() const;

    void mergeClildrenIndexes(Cluster &cluster1, Cluster &cluster2);

    set<int> &getIndexList();

    void setIndexList(set<int> &indexList);

    bool operator < (const Cluster& cluster) const {
        return (distanceBetweenLinks < cluster.distanceBetweenLinks);
    }
};


#endif //PROJEKTMAGISTERSKI_CLASTER_H
