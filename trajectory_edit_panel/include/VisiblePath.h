#ifndef VISIBLE_PATH_H
#define VISIBLE_PATH_H
#include <iostream>
#include <vector>

class VisiblePose
{
public:
    double x;
    double y;
    double z;
    double qw;
    double qx;
    double qy;
    double qz;

    void setPosition(double pos_x, double pos_y, double pos_z)
    {
        x = pos_x;
        y = pos_y;
        z = pos_z;
    }

    void setOrientation(double orient_w, double orient_x, double orient_y, double orient_z)
    {
        qw = orient_w;
        qx = orient_x;
        qy = orient_y;
        qz = orient_z;
    }
};

class VisiblePath
{
public:
    std::vector<VisiblePose> path;

public:
    void setPath(std::vector<VisiblePose> target)
    {
        path.clear();
        path = target;
    }

};

#endif // TEP