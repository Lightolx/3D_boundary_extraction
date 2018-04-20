//
// Created by lightol on 4/19/18.
//

#ifndef INC_3D_BOUNDARY_TRIANGLE_H
#define INC_3D_BOUNDARY_TRIANGLE_H

//
// Created by lightol on 3/16/18.
//

#ifndef DELAUNAY_TRIANGULATION_TRIANGLE_H
#define DELAUNAY_TRIANGULATION_TRIANGLE_H

#include "eigen3/Eigen/Eigen"
#include <opencv2/opencv.hpp>

#include <assert.h>
#include <math.h>

using Point = Eigen::Vector3f;
using Vector = Eigen::Vector3f;

class Triangle
{
public:
    // constructor
    Triangle() {}
    Triangle(const Point &p1, const Point &p2, const Point &p3);
    Triangle(const Triangle &triangle);

    // compute the normal vector f_ and find the circum center of this triangle
    void computeCircumCenter();

    // judge if the inquired point is vertex of this triangle
    bool passPoint(const Point &p) const
    {
        return (p1_ == p || p2_ == p || p3_ == p);
    }

    Point computeSphereCenter(double r, int orient) const;

    static bool findBoundary(const std::vector<cv::Point3f> &pts, std::vector<Point> &points);

    static std::vector<int> boundary(const std::vector<cv::Point2f>& pts);

    Point p1_;
    Point p2_;
    Point p3_;
    Point circumCenter_;     // circum center of this triangle
    float radius_;           // circumCenter radius of this triangle
    Vector f_;               // normal vector of this triangle

    bool isBoundary;
};

#endif //DELAUNAY_TRIANGULATION_TRIANGLE_H

#endif //INC_3D_BOUNDARY_TRIANGLE_H
