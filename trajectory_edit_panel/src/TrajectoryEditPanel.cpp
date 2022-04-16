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
        ros_init();
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
        connect(path_edit_panel.ui->button_rotate, SIGNAL(clicked()), this,
                SLOT(on_button_rotate_clicked()));
            
    }

    void TrajectoryEditPanel::ui_init()
    {
        path_edit_panel.ui->setupUi(this);
        visible_panel.ui->setupUi(this);
        path_edit_panel.ui->hide();
        visible_panel.ui->hide();

        path_edit_panel.ui->table_path->setColumnCount(7);
        QStringList headers;
        headers << QStringLiteral("x") << QStringLiteral("y") << QStringLiteral("z")
                << QStringLiteral("qw") << QStringLiteral("qx") << QStringLiteral("qy")
                << QStringLiteral("qz");
        path_edit_panel.ui->table_path->setHorizontalHeaderLabels(headers);
    }

    void TrajectoryEditPanel::ros_init()
    {
        int argc = 0;
        char **argv = NULL;
        std::string node_name = "cvte_rviz_panel";
        ros::init(argc, argv, node_name);
        nh = new ros::NodeHandle();

        pose_array_publisher = nh->advertise<geometry_msgs::PoseArray>("/tpc_pose_array", 10);
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
            path_edit_panel.ui->table_path->setRowCount(i + 1);
            QTableWidgetItem *item0 = new QTableWidgetItem;
            QTableWidgetItem *item1 = new QTableWidgetItem;
            QTableWidgetItem *item2 = new QTableWidgetItem;
            QTableWidgetItem *item3 = new QTableWidgetItem;
            QTableWidgetItem *item4 = new QTableWidgetItem;
            QTableWidgetItem *item5 = new QTableWidgetItem;
            QTableWidgetItem *item6 = new QTableWidgetItem;

            item0->setText(QString::number(visible_path.path[i].x, 'c', 2));
            item1->setText(QString::number(visible_path.path[i].y, 'c', 2));
            item2->setText(QString::number(visible_path.path[i].z, 'c', 2));
            item3->setText(QString::number(visible_path.path[i].qw, 'c', 2));
            item4->setText(QString::number(visible_path.path[i].qx, 'c', 2));
            item5->setText(QString::number(visible_path.path[i].qy, 'c', 2));
            item6->setText(QString::number(visible_path.path[i].qz, 'c', 2));

            path_edit_panel.ui->table_path->setItem(i, 0, item0);
            path_edit_panel.ui->table_path->setItem(i, 1, item1);
            path_edit_panel.ui->table_path->setItem(i, 2, item2);
            path_edit_panel.ui->table_path->setItem(i, 3, item3);
            path_edit_panel.ui->table_path->setItem(i, 4, item4);
            path_edit_panel.ui->table_path->setItem(i, 5, item5);
            path_edit_panel.ui->table_path->setItem(i, 6, item6);
        }

        if (visible_panel.ui->combox_path_mdl->currentIndex() == 0)
        {
            tablePoseArryPub();
        }
        else
        {
            tableGroundPathPub();
        }
    }
    // Publisher================================================:
    void TrajectoryEditPanel::tablePoseArryPub()
    {
        geometry_msgs::PoseArray msg_path;
        msg_path.header.stamp = ros::Time::now();
        msg_path.header.frame_id = "map";

        for (int i = 0; i < path_edit_panel.ui->table_path->rowCount(); i++)
        {
            geometry_msgs::Pose point_pose;
            point_pose.position.x = path_edit_panel.ui->table_path->item(i, 0)->text().toDouble();
            point_pose.position.y = path_edit_panel.ui->table_path->item(i, 1)->text().toDouble();
            point_pose.position.z = path_edit_panel.ui->table_path->item(i, 2)->text().toDouble();
            point_pose.orientation.w = path_edit_panel.ui->table_path->item(i, 3)->text().toDouble();
            point_pose.orientation.x = path_edit_panel.ui->table_path->item(i, 4)->text().toDouble();
            point_pose.orientation.y = path_edit_panel.ui->table_path->item(i, 5)->text().toDouble();
            point_pose.orientation.z = path_edit_panel.ui->table_path->item(i, 6)->text().toDouble();
            msg_path.poses.push_back(point_pose);
        }

        pose_array_publisher.publish(msg_path);
    }

    void TrajectoryEditPanel::tableGroundPathPub()
    {
    }

    // SLOTS==================================================:
    // mainPanel:
    void TrajectoryEditPanel::on_comboBox_currentIndexChanged(int index)
    {
        if (index == 0)
        {
            path_edit_panel.ui->hide();
            visible_panel.ui->hide();
        }

        else if (index == 1)
        {
            visible_panel.ui->hide();
            path_edit_panel.ui->show();
        }

        else if (index == 2)
        {
            path_edit_panel.ui->hide();
            visible_panel.ui->show();
        }
    }

    // PathEditPanel:
    void TrajectoryEditPanel::on_button_rotate_clicked()
    {
        double angle = path_edit_panel.ui->line_rotate->text().toDouble() / 180.0 * PI_;
        cout << angle << endl;
        Eigen::AngleAxisd abs_rotx(angle, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd abs_roty(angle, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd abs_rotz(angle, Eigen::Vector3d::UnitZ());

        VisiblePath visible_path;

        for (int i = 0; i < path_edit_panel.ui->table_path->rowCount(); i++)
        {
            double x, y, z;
            double qw, qx, qy, qz;
            x = path_edit_panel.ui->table_path->item(i, 0)->text().toDouble();
            y = path_edit_panel.ui->table_path->item(i, 1)->text().toDouble();
            z = path_edit_panel.ui->table_path->item(i, 2)->text().toDouble();
            qw = path_edit_panel.ui->table_path->item(i, 3)->text().toDouble();
            qx = path_edit_panel.ui->table_path->item(i, 4)->text().toDouble();
            qy = path_edit_panel.ui->table_path->item(i, 5)->text().toDouble();
            qz = path_edit_panel.ui->table_path->item(i, 6)->text().toDouble();

            Eigen::Quaterniond qua(qw, qx, qy, qz);
            //自身x轴
            if (path_edit_panel.ui->combox_axies->currentIndex() == 0)
            {
                qua = qua * abs_rotx;
            }
             //自身y轴          
            else if (path_edit_panel.ui->combox_axies->currentIndex() == 1)
            {
                qua = qua * abs_roty;
            }
            //自身z轴
            else if (path_edit_panel.ui->combox_axies->currentIndex() == 2)
            {
                qua = qua * abs_rotz;
            }
            //全局x轴
            else if (path_edit_panel.ui->combox_axies->currentIndex() == 2)
            {
                qua = abs_rotx * qua;
            }
            //全局y轴
            else if (path_edit_panel.ui->combox_axies->currentIndex() == 2)
            {
                qua = abs_roty * qua;
            }
            //全局z轴
            else if (path_edit_panel.ui->combox_axies->currentIndex() == 2)
            {
                qua = abs_rotz * qua;
            }

            qw = qua.w();
            qx = qua.x();
            qy = qua.y();
            qz = qua.z();

            VisiblePose single_pose;
            single_pose.setPosition(x, y, z);
            single_pose.setOrientation(qw, qx, qy, qz);

            visible_path.path.push_back(single_pose);
        }

        addPath2Table(visible_path);
    }
}