#ifndef TEP_H
#define TEP_H


#include <ros/console.h>
#include <rviz/panel.h>
#include "patheditpanel.h"
#include "visiblepanel.h"

#include <stdio.h>
#include <iostream>

#include "qt5_compoment.h"
#include "VisiblePath.h"
 #include <Eigen/Geometry>

 #define PI_ 3.1415926

using namespace std;
namespace rviz_telop_commander
{

    class TrajectoryEditPanel : public rviz::Panel
    {
        Q_OBJECT
    public:
        //components:
        QComboBox *comboBox;
        PathEditPanel path_edit_panel;
        visiblePanel visible_panel;
        double rotate_value;
        double move_x_value;
        double move_y_value;
        double move_z_value;

        //publisher:
        ros::NodeHandle* nh;
        ros::Publisher pose_array_publisher;

        //fuctions_init:
        TrajectoryEditPanel(QWidget *parent = 0);
        void components_init();
        void connect_init();
        void ui_init();
        void ros_init();

        //fuctions_operate:
        void addPath2Table(VisiblePath visible_path);
        VisiblePath createTestCurve();
        void showTablePath();

        //publisher:
        void tablePoseArryPub();
        void tableGroundPathPub();

    protected Q_SLOTS:
        //mainPanel:
         void on_comboBox_currentIndexChanged(int index);

         //pathEditPanel:
         void on_button_rotate_clicked();
    };

}

#endif // TEP
