#ifndef EIGEN_ROTATE_H
#define EIGEN_ROTATE_H

#include <Eigen/Geometry>

#define AXIESX 0
#define AXIESY 1
#define AXIESZ 2
#define GLOBALX 3
#define GLOBALY 4
#define GLOBALZ 5

class EigenRotateManager
{
public:
    static Eigen::Quaterniond createQuat(double qw, double qx, double qy, double qz, double angle, int axis)
    {
        Eigen::Quaterniond qua(qw, qx, qy, qz);
        Eigen::AngleAxisd abs_rotx(angle, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd abs_roty(angle, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd abs_rotz(angle, Eigen::Vector3d::UnitZ());

        //自身X轴
        if (axis == AXIESX)
        {
            qua = qua * abs_rotx;
        }
        //自身y轴
        else if (axis == AXIESY)
        {
            qua = qua * abs_roty;
        }
        //自身z轴
        else if (axis == AXIESZ)
        {
            qua = qua * abs_rotz;
        }
        //全局x轴
        else if (axis == GLOBALX)
        {
            qua = abs_rotx * qua;
        }
        //全局y轴
        else if (axis == GLOBALY)
        {
            qua = abs_roty * qua;
        }
        //全局z轴
        else if (axis == GLOBALZ)
        {
            qua = abs_rotz * qua;
        }

        return qua;
    }
};

#endif