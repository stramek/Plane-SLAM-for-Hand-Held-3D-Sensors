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
void generateMockedPointCloud(PointCloud<T> &point,  array<Point3f, 4> points) {
    std::cout << "Generating point cloud...";
    point.pts.resize(points.size());

    cout<<endl;
    for (size_t i = 0; i < points.size(); i++) {
        point.pts[i].x = points[i].x;
        point.pts[i].y = points[i].y;
        point.pts[i].z = points[i].z;

        cout<<point.pts[i].x<<" "<<point.pts[i].y<<" "<<point.pts[i].z<<endl;
    }

    std::cout << "done\n";
}

template<typename num_t>
void kdtree_demo(const size_t N) {
    PointCloud<num_t> cloud;

    // Generate points:
    generateRandomPointCloud(cloud, N);

    // construct a kd-tree index:
    typedef KDTreeSingleIndexAdaptor<
            L2_Simple_Adaptor<num_t, PointCloud<num_t> >,
            PointCloud<num_t>,
            3 /* dim */
    > my_kd_tree_t;

    my_kd_tree_t index(3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
    index.buildIndex();

    const num_t query_pt[3] = {0.5, 0.5, 0.5};

    // ----------------------------------------------------------------
    // knnSearch():  Perform a search for the N closest points
    // ----------------------------------------------------------------
    {
        size_t num_results = 5;
        std::vector<size_t> ret_index(num_results);
        std::vector<num_t> out_dist_sqr(num_results);

        num_results = index.knnSearch(&query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);

        // In case of less points in the tree than requested:
        ret_index.resize(num_results);
        out_dist_sqr.resize(num_results);

        cout << "knnSearch(): num_results=" << num_results << "\n";
        for (size_t i = 0; i < num_results; i++)
            cout << "idx[" << i << "]=" << ret_index[i] << " dist[" << i << "]=" << out_dist_sqr[i] << endl;
        cout << "\n";
    }

    // ----------------------------------------------------------------
    // radiusSearch():  Perform a search for the N closest points
    // ----------------------------------------------------------------
    {
        const num_t search_radius = static_cast<num_t>(0.1);
        std::vector<std::pair<size_t, num_t> > ret_matches;

        nanoflann::SearchParams params;
        //params.sorted = false;

        const size_t nMatches = index.radiusSearch(&query_pt[0], search_radius, ret_matches, params);

        cout << "radiusSearch(): radius=" << search_radius << " -> " << nMatches << " matches\n";
        for (size_t i = 0; i < nMatches; i++)
            cout << "idx[" << i << "]=" << ret_matches[i].first << " dist[" << i << "]=" << ret_matches[i].second
                 << endl;
        cout << "\n";
    }
}

Point getNearestPoint() {

}

int main(int argc, char **argv) {
// Randomize Seed
    srand(time(NULL));
    //kdtree_demo<float>(4);
//    kdtree_demo<double>(100000);


    int N = 10000;
    PointCloud<float> cloud;

    // Generate points:
    //generateRandomPointCloud(cloud, N);

    array<Point3f, 4> points = {
            Point3f(0, 0, 0),
            Point3f(2, 3, 0),
            Point3f(3, 2, 0),
            Point3f(3, 1, 0)
    };
    generateMockedPointCloud(cloud, points);

    // construct a kd-tree index:
    typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<float, PointCloud<float>>, PointCloud<float>, 3 /* dim */> my_kd_tree_t;
    my_kd_tree_t index(3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));

    index.buildIndex();

    const float query_pt[3] = {0.0, 0.0, 0.0};

    // ----------------------------------------------------------------
    // knnSearch():  Perform a search for the N closest points
    // ----------------------------------------------------------------
    {
        size_t num_results = 4;
        std::vector<size_t> ret_index(num_results);
        std::vector<float> out_dist_sqr(num_results);

        num_results = index.knnSearch(query_pt, num_results, &ret_index[0], &out_dist_sqr[0]);

        // In case of less points in the tree than requested:
        ret_index.resize(num_results);
        out_dist_sqr.resize(num_results);

        cout << "knnSearch(): num_results=" << num_results << "\n";
        for (size_t i = 0; i < num_results; i++)
            cout << "idx[" << i << "]=" << ret_index[i] << " dist[" << i << "]=" << out_dist_sqr[i] << endl;
        cout << "\n";
    }


    return 0;
}
