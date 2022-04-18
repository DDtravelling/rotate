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
#include "rosInit.h"
#include "eigenRotate.h"

#define PI_ 3.1415926
#define UI_PATH path_edit_panel.ui
#define UI_VISIBLE visible_panel.ui
using namespace std;
namespace rviz_telop_commander
{

    class TrajectoryEditPanel : public rviz::Panel
    {
        Q_OBJECT
    public:
        // components:
        QComboBox *comboBox;
        PathEditPanel path_edit_panel;
        visiblePanel visible_panel;
        double rotate_value = 0;
        double move_x_value = 0;
        double move_y_value = 0;
        double move_z_value = 0;
        ros_init_treatment::rosInitManager ros_manager;

        // fuctions_init:
        TrajectoryEditPanel(QWidget *parent = 0);
        void components_init();
        void connect_init();
        void ui_init();

        // fuctions_operate:
        void addPath2Table(VisiblePath visible_path);
        VisiblePath createTestCurve();
        void showTablePath();
        VisiblePath rotateTable(double angle);
        VisiblePath moveTable(double x,double y , double z);

        // publisher:
        void tablePoseArryPub();
        void tableGroundPathPub();

    protected Q_SLOTS:
        // mainPanel:
        void on_comboBox_currentIndexChanged(int index);

        // pathEditPanel:
        void on_button_rotate_clicked_d();
        void on_button_subx_clicked_d();
        void on_button_addx_clicked_d();
        void on_button_suby_clicked_d();
        void on_button_addy_clicked_d();
        void on_button_addz_clicked_d();
        void on_button_subz_clicked_d();
    };

}

#endif // TEP
