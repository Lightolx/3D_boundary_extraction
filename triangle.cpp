//
// Created by lightol on 4/19/18.
//

#include "triangle.h"

Triangle::Triangle(const Point &p1, const Point &p2, const Point &p3)
{
    p1_ = p1;
    p2_ = p2;
    p3_ = p3;
    isBoundary = false;

    computeCircumCenter();
}

Triangle::Triangle(const Triangle &triangle)
{
    p1_ = triangle.p1_;
    p2_ = triangle.p2_;
    p3_ = triangle.p3_;

    isBoundary = triangle.isBoundary;
    circumCenter_ = triangle.circumCenter_;
    radius_ = triangle.radius_;
    f_ = triangle.f_;
}

void Triangle::computeCircumCenter()
{
    Point D = (p1_ + p2_)/2;
    Point E = (p2_ + p3_)/2;
    Vector a = p2_ - p1_;
    Vector b = p3_ - p2_;
    f_ = a.cross(b);   // vector f_ is orthogonal to plane p1p2p3
    f_.normalize();

    // vector d is orthogonal to vector a and f, i.e., d is in plane p1p2p3 and orthogonal to
    // vector a, i.e., its p1p2's midperpendicular
    Vector d = a.cross(f_);
    Vector e = b.cross(f_);  // so is e as d

    d.normalize();
    e.normalize();
    // Now, solve the equation D + x*a = E + y*b, i.e., the circumCenter of this triangle
    Eigen::Matrix2f de;
    de << d[0], -e[0], d[1], -e[1];
    Eigen::Vector2f xy = de.inverse()*((E - D).topRows<2>());
    circumCenter_ = D + xy[0]*d;
    radius_ = (circumCenter_ - p1_).norm();
}

Point Triangle::computeSphereCenter(double r, int orient) const
{
    // solve the equation (F + x*f - p1).norm() = r, i.e., |p1F|^2 + x^2 = r^2
    double x = std::sqrt(pow(r, 2) - pow(radius_, 2));

    return circumCenter_ + orient * x * f_;
}

bool Triangle::findBoundary(const std::vector<cv::Point3f> &points, std::vector<Point> &pts)
{
    int ptNum = points.size();
    std::vector<cv::Point2f> ptxs;
    ptxs.reserve(ptNum);
    std::vector<cv::Point2f> ptys;
    ptys.reserve(ptNum);
    std::vector<cv::Point2f> ptzs;
    ptzs.reserve(ptNum);

    for (const cv::Point3f &pt : points)
    {
        ptxs.push_back(cv::Point2f(pt.x, pt.z));
        ptys.push_back(cv::Point2f(pt.x, pt.y));
        ptzs.push_back(cv::Point2f(pt.y, pt.z));
    }

    std::vector<int> Kx = Triangle::boundary(ptxs);
    std::vector<int> Ky = Triangle::boundary(ptys);
    std::vector<int> Kz = Triangle::boundary(ptzs);
    Kx.insert(Kx.end(), Ky.begin(), Ky.end());
    Kx.insert(Kx.end(), Kz.begin(), Kz.end());
    std::sort(Kx.begin(), Kx.end());
    Kx.erase(std::unique(Kx.begin(), Kx.end()), Kx.end());

    int pointNum = Kx.size();
    pts.reserve(pointNum);

    for (int id : Kx)
    {
        pts.push_back(Point(points[id].x, points[id].y, points[id].z));
    }
}

std::vector<int> Triangle::boundary(const std::vector<cv::Point2f> &pts)
{
    std::vector<cv::Point2f> boundaryPts;
    cv::convexHull(pts, boundaryPts);
    int ptNum = pts.size();
    std::vector<int> ids;
    ids.reserve(ptNum);

    for (const cv::Point2f &bPt : boundaryPts)
    {
        for (int i = 0; i < ptNum; ++i)
        {
            if (bPt == pts[i])
            {
                ids.push_back(i);
                break;
            }
        }
    }

    return ids;
}