//
// Created by stramek on 15.07.17.
//

#include "include/models/PosOrient.h"


PosOrient::PosOrient(const Vector3d &position, const Vector4d &orientation) : position(position),
                                                                              orientation(orientation) {

}

void PosOrient::setPosOrient(const g2o::Vector7d &posOrient) {
    position[0] = posOrient[0];
    position[1] = posOrient[1];
    position[2] = posOrient[2];

    orientation[0] = posOrient[3];
    orientation[1] = posOrient[4];
    orientation[2] = posOrient[5];
    orientation[3] = posOrient[6];
}

const void PosOrient::print() const {
    cout << "x: " << position[0] << " y: " << position[1] << " z: " << position[2] << endl;
    cout << "q1: " << orientation[0] << " q2: " << orientation[1] << " q3: " << orientation[2] << " q4: "
         << orientation[3] << endl;
    const Vector3d euler = fromRotationMat(getQuaternion().toRotationMatrix());
    cout<<"alpha: "<<euler(0)<<" betta: "<<euler(1)<<" gamma: "<<euler(2)<<endl;
}

void PosOrient::printDiff(const PosOrient &posOrient) {
    cout << "x: " << abs(position[0] - posOrient.position[0])
         << " y: " << abs(position[1] - posOrient.position[1])
         << " z: " << abs(position[2] - posOrient.position[2]) << endl;
    cout << "q1: " << abs(orientation[0] - posOrient.orientation[0])
         << " q2: " << abs(orientation[1] - posOrient.orientation[1])
         << " q3: " << abs(orientation[2] - posOrient.orientation[2])
         << " q4: " << abs(orientation[3] - posOrient.orientation[3]) << endl;
}

PosOrient::PosOrient() {

}

Quaterniond PosOrient::getQuaternion() const {
    Quaterniond q(orientation[3], orientation[0], orientation[1], orientation[2]);
    return q;
}

Vector3d PosOrient::getPosition() const {
    return position;
}

pair<Vector3d, Vector3d> PosOrient::minus(PosOrient &posOrient) {
    Vector3d positionDif = position - posOrient.getPosition();
    Matrix<double, 3, 3> rotDifMat = posOrient.getQuaternion().toRotationMatrix().inverse() * getQuaternion().toRotationMatrix();
    Vector3d rotationDiff = fromRotationMat(rotDifMat);
    for (int i = 0; i < 3; ++i) {
        rotationDiff(i) = abs(rotationDiff(i));
        positionDif(i) = abs(positionDif(i));
    }
    return pair<Vector3d, Vector3d>(positionDif, rotationDiff);
}

/// compute roll/pitch/yaw from rotation matrix
Vector3d PosOrient::fromRotationMat(const Matrix<double, 3, 3>& pose) const {
    Vector3d rpy(Vector3d::Identity());
    rpy.x() = atan2(pose(2,1), pose(2,2)) * 180.0 / (double) M_PI;
    rpy.y() = -asin(pose(2,0)) * 180.0 / (double) M_PI;
    rpy.z() = atan2(pose(1,0), pose(0,0)) * 180.0 / (double) M_PI;
    return rpy;
}