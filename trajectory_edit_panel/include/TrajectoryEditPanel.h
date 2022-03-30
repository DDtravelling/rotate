#ifndef TEP_H
#define TEP_H

#include <ros/console.h>
#include <rviz/panel.h>

#include <stdio.h>
#include <iostream>
using namespace std;
namespace rviz_telop_commander
{

    class TrajectoryEditPanel : public rviz::Panel
    {
        Q_OBJECT
    public:
    public:
        TrajectoryEditPanel(QWidget *parent = 0);

    protected Q_SLOTS:
    };

}

#endif // TEP
