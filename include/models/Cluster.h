//
// Created by mordimer on 11.03.17.
//

#ifndef PROJEKTMAGISTERSKI_CLASTER_H
#define PROJEKTMAGISTERSKI_CLASTER_H

class Cluster {
private:
    int firstLinkIndex;
    int secondLinkIndex;
    float distanceBetweenLinks;
public:
    void setFirstLinkIndex(int firstLinkIndex);

    void setSecondLinkIndex(int secondLinkIndex);

    void setDistanceBetweenLinks(float distanceBetweenLinks);

    int getFirstLinkIndex() const;

    int getSecondLinkIndex() const;

    float getDistanceBetweenLinks() const;
};


#endif //PROJEKTMAGISTERSKI_CLASTER_H
