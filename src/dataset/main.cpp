//
//  main.cpp
//  Projekt Magisterski
//
//  Created by Marcin Stramowski on 15.12.2016.
//  Copyright © 2016 Marcin Stramowski. All rights reserved.
//

#include "include/dataset/main.h"
#include "include/clustering/nanoflann.h"
#include <math.h>
#include <unordered_set>
#include <include/clustering/Cluster.h>

using namespace nanoflann;

// This is an example of a custom data set class
template<typename T>
struct PointCloud {
    struct Point {
        T x, y, z;
    };

    std::vector<Point> pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
    inline T kdtree_distance(const T *p1, const size_t idx_p2, size_t /*size*/) const {
        const T d0 = p1[0] - pts[idx_p2].x;
        const T d1 = p1[1] - pts[idx_p2].y;
        const T d2 = p1[2] - pts[idx_p2].z;
        return sqrt(d0 * d0 + d1 * d1 + d2 * d2);
    }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline T kdtree_get_pt(const size_t idx, int dim) const {
        if (dim == 0) return pts[idx].x;
        else if (dim == 1) return pts[idx].y;
        else return pts[idx].z;
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template<class BBOX>
    bool kdtree_get_bbox(BBOX & /*bb*/) const { return false; }

};

template<typename T>
void generateRandomPointCloud(PointCloud<T> &point, const size_t N, const T max_range = 10) {
    std::cout << "Generating " << N << " point cloud...";
    point.pts.resize(N);
    for (size_t i = 0; i < N; i++) {
        point.pts[i].x = max_range * (rand() % 1000) / T(1000);
        point.pts[i].y = max_range * (rand() % 1000) / T(1000);
        point.pts[i].z = max_range * (rand() % 1000) / T(1000);
    }

    std::cout << "done\n";
}

template<typename T>
void generateMockedPointCloud(PointCloud<T> &point, vector<Point3f> points) {
    std::cout << "Generating point cloud...";
    point.pts.resize(points.size());

    cout << endl;
    for (size_t i = 0; i < points.size(); i++) {
        point.pts[i].x = points[i].x;
        point.pts[i].y = points[i].y;
        point.pts[i].z = points[i].z;

        cout << point.pts[i].x << " " << point.pts[i].y << " " << point.pts[i].z << endl;
    }

    std::cout << "done\n";
}

typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<float, PointCloud<float>>, PointCloud<float>, 3 /* dim */> my_kd_tree_t;

Point3f getNearestPoint(const my_kd_tree_t &tree, const Point3f &point, const vector<Point3f> &points) {
    float query_pt[3] = {point.x, point.y, point.z};

    size_t index;
    float distance;

    tree.knnSearch(query_pt, 1, &index, &distance);
    return points[index];
}

template<typename T>
void updatePoints(PointCloud<T> &cloud, vector<Point3f> points) {
    generateMockedPointCloud(cloud, points);
    my_kd_tree_t index(3, cloud, KDTreeSingleIndexAdaptorParams());
    index.buildIndex();
}

int main(int argc, char **argv) {
    PointCloud<float> cloud;

    vector<Point3f> points = {
            Point3f(0, 0, 0),
            Point3f(2, 3, 0),
            Point3f(3, 2, 0),
            Point3f(3, 1, 0)
    };

    generateMockedPointCloud(cloud, points);
    my_kd_tree_t index(3, cloud, KDTreeSingleIndexAdaptorParams());
    index.buildIndex();
    //cout << "Nearest point: " << getNearestPoint(index, Point3f(3.2, 1.8, 0), points) << endl;

    unordered_set<Cluster> ws;
    for (Point3f point : points) {
        ws.insert(Cluster(point));
    }

    unordered_set<Cluster> newWork;
    while (true) {
        for (Cluster p : ws) {
            if (p.hasCluster()) continue;
            Cluster q = Cluster(getNearestPoint(index, p.getPoint(), points));
            // if (q == null) break;
            Cluster r = Cluster(getNearestPoint(index, q.getPoint(), points));
            if (p == r) {
                Cluster *e = p.merge(&q);
                newWork.insert(*e);
            } else {
                newWork.insert(p);
            }
        }
        if (newWork.size() == 1) break;
        for (Cluster cluster : newWork) {
            ws.insert(cluster);
        }

        vector<Point3f> tmpNewWorkPoints;
        for (Cluster c : newWork) {
            tmpNewWorkPoints.push_back(c.getPoint());
        }
        updatePoints(cloud, tmpNewWorkPoints);
        newWork.clear();
    }

    return 0;
}
