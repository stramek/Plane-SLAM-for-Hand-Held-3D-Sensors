//
// Created by stramek on 15.07.17.
//

#include "include/models/PosOrient.h"


PosOrient::PosOrient(const Vector3d &position, const Vector4d &orientation) : position(position),
                                                                              orientation(orientation) {

}

void PosOrient::setPosOrient(const Vector7d &posOrient) {
    position[0] = posOrient[0];
    position[1] = posOrient[1];
    position[2] = posOrient[2];

    orientation[0] = posOrient[3];
    orientation[1] = posOrient[4];
    orientation[2] = posOrient[5];
    orientation[3] = posOrient[6];
}

void PosOrient::print() {
    cout<<"x: "<<position[0]<<" y: "<<position[1]<<" z: "<<position[2]<<endl;
    cout<<"q1: "<<orientation[0]<<" q2: "<<orientation[1]<<" q3: "<<orientation[2]<<" q4: "<<orientation[3] << endl;
}

void PosOrient::printDiff(const PosOrient &posOrient) {
    cout<<"x: "<<abs(position[0] - posOrient.position[0])
        <<" y: "<<abs(position[1] - posOrient.position[1])
        <<" z: "<<abs(position[2] - posOrient.position[2])<<endl;
    cout<<"q1: "<<abs(orientation[0] - posOrient.orientation[0])
        <<" q2: "<<abs(orientation[1] - posOrient.orientation[1])
        <<" q3: "<<abs(orientation[2] - posOrient.orientation[2])
        <<" q4: "<<abs(orientation[3] - posOrient.orientation[3])<<endl;
}
