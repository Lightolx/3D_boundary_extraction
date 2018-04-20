#include <iostream>
#include <fstream>
#include "triangle.h"

using std::cout;
using std::endl;

int main()
{
    // Step0: Read in raw points
    std::ifstream fin("/media/psf/Home/Documents/MATLAB_codes/3D_boundary/pts.txt");
    std::string ptline;
    double x, y, z;
    std::vector<cv::Point3f> pts;

    while (getline(fin, ptline))
    {
        std::stringstream ss(ptline);
        ss >> x >> y >> z;
        pts.push_back(cv::Point3f(x,y,z));
    }

    // Step1: Project all points to plane X0Z and X0Y, find the boundary points
    std::vector<Point> points;
    Triangle::findBoundary(pts, points);

    // Step1: I only want to find a convex hull, not an alpha shape, so I will set the radius of the chop
    //        sphere big enough to contain all tiangles
    int ptNum = points.size();
    std::vector<Triangle> triangles;
    triangles.reserve(ptNum*(ptNum-1)*(ptNum-2) / 6);
    float maxR = 0;

    for (int i = 0; i < ptNum; ++i)
    {
        for (int j = i+1; j < ptNum; ++j)
        {
            for (int k = j+1; k < ptNum; ++k)
            {
                Triangle triangle(points[i], points[j], points[k]);
                triangles.push_back(triangle);

                if (triangle.radius_ > maxR)
                {
                    maxR = triangle.radius_;
                }
            }
        }
    }

    std::cout << "maxR is " << maxR << std::endl;
    // Step2: for every triangle, use a sphere whose radius is maxR to touch it, inspect if there is other
    //        points in this sphere, if not, it is a boundary triangle
    float sphereR = 0.0;

    for (Triangle &triangle : triangles)
    {
        bool sphere1 = false;     // sphere1 has other points
        bool sphere2 = false;     // sphere2 has other points

        Point sphereCenter1 = triangle.computeSphereCenter(4*maxR, 1);
        Point sphereCenter2 = triangle.computeSphereCenter(4*maxR, -1);
        sphereR = (sphereCenter1 - triangle.p1_).norm();

        for (const Point &pt : points)
        {
            if ((pt - sphereCenter1).norm() < sphereR && !triangle.passPoint(pt))
            {
                sphere1 = true;
                break;
            }
        }

        for (const Point &pt : points)
        {
            if ((pt - sphereCenter2).norm() < sphereR && !triangle.passPoint(pt))
            {
                sphere2 = true;
                break;
            }
        }

        if ((!sphere1) || (!sphere2))
        {
            triangle.isBoundary = true;
        }
    }

    // Step4: Output the result
    std::ofstream fout1("/media/psf/Home/Documents/MATLAB_codes/3D_boundary/points.txt");
    for (const Point &pt : points)
    {
        fout1 << pt.x() << " " << pt.y() << " " << pt.z() << endl;
    }
    fout1.close();

    std::ofstream fout("/media/psf/Home/Documents/MATLAB_codes/3D_boundary/pt1.txt");
    int id1(0), id2(0), id3(0);

    for (const Triangle &tri : triangles)
    {
        if (tri.isBoundary)
        {
            for (int i = 0; i < ptNum; ++i)
            {
                if (points[i] == tri.p1_)
                {
                    id1 = i;
                    break;
                }
            }

            for (int i = 0; i < ptNum; ++i)
            {
                if (points[i] == tri.p2_)
                {
                    id2 = i;
                    break;
                }
            }

            for (int i = 0; i < ptNum; ++i)
            {
                if (points[i] == tri.p3_)
                {
                    id3 = i;
                    break;
                }
            }

            fout << id1+1 << " " << id2+1 << " " << id3+1 << endl;
        }
    }

    fout.close();
}