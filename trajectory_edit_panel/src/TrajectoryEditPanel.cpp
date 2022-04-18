#include "TrajectoryEditPanel.h"
#include <pluginlib/class_list_macros.h>
#include "ui_patheditpanel.h"
#include "ui_visiblepanel.h"
PLUGINLIB_EXPORT_CLASS(rviz_telop_commander::TrajectoryEditPanel, rviz::Panel);
namespace rviz_telop_commander
{
    // init fuctions:
    TrajectoryEditPanel::TrajectoryEditPanel(QWidget *parent)
        : rviz::Panel(parent)
    {
        components_init();
        ui_init();
        connect_init();

        VisiblePath tst_path = createTestCurve();
        addPath2Table(tst_path);
    }

    void TrajectoryEditPanel::components_init()
    {
        // form:
        comboBox = new QComboBox(this);
        comboBox->setObjectName(QStringLiteral("comboBox"));
        comboBox->setGeometry(QRect(0, 0, 101, 25));

        // text:
        comboBox->clear();
        comboBox->insertItems(0, QStringList()
                                     << QApplication::translate("RvizPanel", "\345\216\237\345\247\213\351\235\242\346\235\277", Q_NULLPTR)
                                     << QApplication::translate("RvizPanel", "\350\275\250\350\277\271\347\224\237\346\210\220\351\235\242\346\235\277", Q_NULLPTR)
                                     << QApplication::translate("RvizPanel", "\345\217\257\350\247\206\345\214\226\350\260\203\346\225\264\351\235\242\346\235\277", Q_NULLPTR));
    }

    void TrajectoryEditPanel::connect_init()
    {
        connect(this->comboBox, SIGNAL(currentIndexChanged(int)), this,
                SLOT(on_comboBox_currentIndexChanged(int)));

        // path_edit_panel
        connect(UI_PATH->button_rotate, SIGNAL(clicked()), this,
                SLOT(on_button_rotate_clicked_d()));
        connect(UI_PATH->button_subx, SIGNAL(clicked()), this,
                SLOT(on_button_subx_clicked_d()));
        connect(UI_PATH->button_addx, SIGNAL(clicked()), this,
                SLOT(on_button_addx_clicked_d()));
        connect(UI_PATH->button_suby, SIGNAL(clicked()), this,
                SLOT(on_button_suby_clicked_d()));
        connect(UI_PATH->button_addy, SIGNAL(clicked()), this,
                SLOT(on_button_addy_clicked_d()));
        connect(UI_PATH->button_subz, SIGNAL(clicked()), this,
                SLOT(on_button_subz_clicked_d()));
        connect(UI_PATH->button_addz, SIGNAL(clicked()), this,
                SLOT(on_button_addz_clicked_d()));

        // visible_panel
        connect(UI_VISIBLE->combox_path_mdl, SIGNAL(currentIndexChanged(int)), this,
                SLOT(combox_path_mdl_index_change(int)));
    }

    void TrajectoryEditPanel::ui_init()
    {
        UI_PATH->setupUi(this);
        UI_VISIBLE->setupUi(this);
        UI_PATH->hide();
        UI_VISIBLE->hide();
        UI_PATH->table_path->setColumnCount(7);
        QStringList headers;
        headers << QStringLiteral("x") << QStringLiteral("y") << QStringLiteral("z")
                << QStringLiteral("qw") << QStringLiteral("qx") << QStringLiteral("qy")
                << QStringLiteral("qz");
        UI_PATH->table_path->setHorizontalHeaderLabels(headers);
        UI_PATH->line_rotate->setText(QString("10"));
        UI_PATH->line_move_step->setText(QString("0.01"));
    }

    // OPERATE:
    VisiblePath TrajectoryEditPanel::createTestCurve()
    {
        VisiblePath result_path;
        for (double i = 0; i < 200; i++)
        {
            VisiblePose single_pose;
            single_pose.x = i / 100;
            single_pose.y = 1.5;
            single_pose.z = 0;

            single_pose.qw = 1;
            single_pose.qx = 0;
            single_pose.qy = 0;
            single_pose.qz = 0.6;

            result_path.path.push_back(single_pose);
        }

        return result_path;
    }

    void TrajectoryEditPanel::addPath2Table(VisiblePath visible_path)
    {
        for (int i = 0; i < visible_path.path.size(); i++)
        {
            UI_PATH->table_path->setRowCount(i + 1);
            setTableRow(visible_path.path[i], UI_PATH->table_path, i);
        }

        if (UI_VISIBLE->combox_path_mdl->currentIndex() == 0)
        {
            tablePoseArryPub();
        }
        else
        {
            tableGroundPathPub();
        }
    }

    VisiblePath TrajectoryEditPanel::rotateTable(double angle)
    {
        VisiblePath visible_path;
        for (int i = 0; i < UI_PATH->table_path->rowCount(); i++)
        {
            double x, y, z;
            double qw, qx, qy, qz;
            readTableRow(x, y, z, qw, qx, qy, qz, UI_PATH->table_path, i);

            Eigen::Quaterniond qua;
            int axis = UI_PATH->combox_axies->currentIndex();
            qua = EigenRotateManager::createQuat(qw, qx, qy, qz, angle, axis);
            VisiblePose single_pose;
            single_pose.setPosition(x, y, z);
            single_pose.setOrientation(qua.w(), qua.x(), qua.y(), qua.z());

            visible_path.path.push_back(single_pose);
        }
        return visible_path;
    }

    VisiblePath TrajectoryEditPanel::moveTable(double addx, double addy, double addz)
    {
        VisiblePath visible_path;
        for (int i = 0; i < UI_PATH->table_path->rowCount(); i++)
        {
            double x, y, z;
            double qw, qx, qy, qz;
            readTableRow(x, y, z, qw, qx, qy, qz, UI_PATH->table_path, i);
            x += addx, y += addy, z += addz;
            VisiblePose single_pose;
            single_pose.setPosition(x, y, z);
            single_pose.setOrientation(qw, qx, qy, qz);
            visible_path.path.push_back(single_pose);
        }
        return visible_path;
    }

    // Publisher================================================:
    void TrajectoryEditPanel::tablePoseArryPub()
    {
        geometry_msgs::PoseArray msg_path;
        msg_path.header.stamp = ros::Time::now();
        msg_path.header.frame_id = "map";

        for (int i = 0; i < UI_PATH->table_path->rowCount(); i++)
        {
            geometry_msgs::Pose point_pose;
            readTableRow(point_pose, UI_PATH->table_path, i);
            msg_path.poses.push_back(point_pose);
        }
        ros_manager.ground_posearry_pub(msg_path);
    }

    void TrajectoryEditPanel::tablePoseArryPubVoid()
    {
        geometry_msgs::PoseArray msg_path;
        msg_path.header.stamp = ros::Time::now();
        msg_path.header.frame_id = "map";
        ros_manager.ground_posearry_pub(msg_path);
    }

    void TrajectoryEditPanel::tableGroundPathPub()
    {
        nav_msgs::Path msg_path;
        msg_path.header.stamp = ros::Time::now();
        msg_path.header.frame_id = "/map";
        double x, y, z, qw, qx, qy, qz;

        for (int i = 0; i < UI_PATH->table_path->rowCount(); i++)
        {
            readTableRow(x, y, z, qw, qx, qy, qz, UI_PATH->table_path, i);
            geometry_msgs::PoseStamped this_pose_stamped;
            this_pose_stamped.pose.position.x = x, this_pose_stamped.pose.position.y = y, this_pose_stamped.pose.position.z = z;
            this_pose_stamped.pose.orientation.x = qx, this_pose_stamped.pose.orientation.y = qy, this_pose_stamped.pose.orientation.z = qz, this_pose_stamped.pose.orientation.w = qw;
            this_pose_stamped.header.stamp = ros::Time::now();
            this_pose_stamped.header.frame_id = "/map";
            msg_path.poses.push_back(this_pose_stamped);
        }
        ros_manager.ground_path_pub(msg_path);
    }

    void TrajectoryEditPanel::tableGroundPathPubVoid()
    {
        nav_msgs::Path msg_path;
        msg_path.header.stamp = ros::Time::now();
        msg_path.header.frame_id = "/map";
        ros_manager.ground_path_pub(msg_path);
    }

    // SLOTS==================================================:
    // mainPanel:
    void TrajectoryEditPanel::on_comboBox_currentIndexChanged(int index)
    {
        if (index == 0)
        {
            UI_PATH->hide();
            UI_VISIBLE->hide();
        }

        else if (index == 1)
        {
            UI_VISIBLE->hide();
            UI_PATH->show();
        }

        else if (index == 2)
        {
            UI_PATH->hide();
            UI_VISIBLE->show();
        }
    }

    // PathEditPanel:
    void TrajectoryEditPanel::on_button_rotate_clicked_d()
    {
        double angle = UI_PATH->line_rotate->text().toDouble() / 180.0 * PI_;
        int axis = UI_PATH->combox_axies->currentIndex();
        rotate_value[axis] += UI_PATH->line_rotate->text().toDouble();
        VisiblePath visible_path = rotateTable(angle);
        UI_PATH->label_rotate_value->setText(QString::number(rotate_value[axis], 'c', 2));
        addPath2Table(visible_path);
    }

    void TrajectoryEditPanel::on_button_subx_clicked_d()
    {
        double move_cnt = UI_PATH->line_move_step->text().toDouble();
        VisiblePath visible_path = moveTable(-move_cnt, 0, 0);
        move_x_value += move_cnt;
        UI_PATH->label_move_value->setText(QString::number(move_x_value, 'c', 2) +
                                           QString(",") + QString::number(move_y_value, 'c', 2) + QString(",") + QString::number(move_z_value, 'c', 2));
        addPath2Table(visible_path);
    }
    void TrajectoryEditPanel::on_button_addx_clicked_d()
    {
        double move_cnt = UI_PATH->line_move_step->text().toDouble();
        VisiblePath visible_path = moveTable(move_cnt, 0, 0);
        move_x_value += move_cnt;
        UI_PATH->label_move_value->setText(QString::number(move_x_value, 'c', 2) +
                                           QString(",") + QString::number(move_y_value, 'c', 2) + QString(",") + QString::number(move_z_value, 'c', 2));
        addPath2Table(visible_path);
    }
    void TrajectoryEditPanel::on_button_suby_clicked_d()
    {
        double move_cnt = UI_PATH->line_move_step->text().toDouble();
        VisiblePath visible_path = moveTable(0, -move_cnt, 0);
        move_y_value += move_cnt;
        UI_PATH->label_move_value->setText(QString::number(move_x_value, 'c', 2) +
                                           QString(",") + QString::number(move_y_value, 'c', 2) + QString(",") + QString::number(move_z_value, 'c', 2));
        addPath2Table(visible_path);
    }
    void TrajectoryEditPanel::on_button_addy_clicked_d()
    {
        double move_cnt = UI_PATH->line_move_step->text().toDouble();
        VisiblePath visible_path = moveTable(0, move_cnt, 0);
        move_y_value += move_cnt;
        UI_PATH->label_move_value->setText(QString::number(move_x_value, 'c', 2) +
                                           QString(",") + QString::number(move_y_value, 'c', 2) + QString(",") + QString::number(move_z_value, 'c', 2));
        addPath2Table(visible_path);
    }
    void TrajectoryEditPanel::on_button_addz_clicked_d()
    {
        double move_cnt = UI_PATH->line_move_step->text().toDouble();
        VisiblePath visible_path = moveTable(0, 0, move_cnt);
        move_z_value += move_cnt;
        UI_PATH->label_move_value->setText(QString::number(move_x_value, 'c', 2) +
                                           QString(",") + QString::number(move_y_value, 'c', 2) + QString(",") + QString::number(move_z_value, 'c', 2));
        addPath2Table(visible_path);
    }
    void TrajectoryEditPanel::on_button_subz_clicked_d()
    {
        double move_cnt = UI_PATH->line_move_step->text().toDouble();
        VisiblePath visible_path = moveTable(0, 0, -move_cnt);
        move_z_value += move_cnt;
        UI_PATH->label_move_value->setText(QString::number(move_x_value, 'c', 2) +
                                           QString(",") + QString::number(move_y_value, 'c', 2) + QString(",") + QString::number(move_z_value, 'c', 2));
        addPath2Table(visible_path);
    }

    // visible Panel:
    void TrajectoryEditPanel::combox_path_mdl_index_change(int index)
    {
        if (index == 0)
        {
            tableGroundPathPubVoid();
            tablePoseArryPub();
        }
        else if (index == 1)
        {
             tablePoseArryPubVoid();
             tableGroundPathPub();
        }
    }
}