#include "cloud.h"

// explicit point(const point & pt)
// {
//     _point(pt.x(), pt.y(), pt.z());
//     _intensity = pt.i();
// }

void point::operator=(const point & other)
{
    _point = other.AsEigenVector();
    _intensity = other.i();
}

void point::operator=(const Eigen::Vector3f& other) 
{
    _point = other;
}

bool point::operator==(const point & other) const
{
     return x() == other.x() && y() == other.y() &&
         z() == other.z() && i() == other.i();
}

//  bounding box with center and orientation
BBox::BBox(const point & x1,
        const point & x2,
        const point & x3,
        const point & x4)
{
    points[0] = x1;
    points[1] = x2;
    points[2] = x3;
    points[3] = x4;

    // 中心点
    pose.position.x = 0.5 * (x1.x() + x3.x());
    pose.position.y = 0.5 * (x1.y() + x3.y());
    pose.position.z = 0.0;

    // 角度
    pose.yaw = (x1.y() - x2.y()) / (x1.x() - x2.x() + 1e-6);

    // 维度
    dimensions.x = sqrt((x1.x() - x2.x()) * (x1.x() - x2.x()) + (x1.y() - x2.y()) * (x1.y() - x2.y()));
    dimensions.y = sqrt((x3.x() - x2.x()) * (x3.x() - x2.x()) + (x3.y() - x2.y()) * (x3.y() - x2.y()));
    dimensions.z = 0.0;
}

BBox::BBox(const std::vector<point> & bbox)
{
    BBox(bbox[0], bbox[1], bbox[2], bbox[3]);
}

BBox::BBox(const Cloud & bbox)
{
    BBox(bbox[0], bbox[1], bbox[2], bbox[3]);
}