#ifndef PATHEDITPANEL_H
#define PATHEDITPANEL_H

#include <QWidget>
#include <QTableWidget>
#include <geometry_msgs/PoseArray.h>
#include "VisiblePath.h"
#include <iostream>

void readTableRow(geometry_msgs::Pose &point_pose, QTableWidget *table_path, int row);
void readTableRow(double &x, double &y, double &z, double &qw, double &qx, double &qy, double &qz, QTableWidget *table_path, int row);
void setTableRow(const VisiblePose &point_pose, QTableWidget *&table_path, int row);
void setTableRow(const double &x, const double &y, const double &z, const double &qw, const double &qx, const double &qy, const double &qz, QTableWidget *&table_path, int row);
namespace Ui
{
    class PathEditPanel;
}

class PathEditPanel : public QWidget
{
    Q_OBJECT

public:
    explicit PathEditPanel(QWidget *parent = 0);
    ~PathEditPanel();

    Ui::PathEditPanel *ui;
};

#endif // PATHEDITPANEL_H
