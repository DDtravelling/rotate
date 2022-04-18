#include "patheditpanel.h"
#include "ui_patheditpanel.h"

void readTableRow(geometry_msgs::Pose &point_pose, QTableWidget *table_path, int row)
{
    point_pose.position.x = table_path->item(row, 0)->text().toDouble();
    point_pose.position.y = table_path->item(row, 1)->text().toDouble();
    point_pose.position.z = table_path->item(row, 2)->text().toDouble();
    point_pose.orientation.w = table_path->item(row, 3)->text().toDouble();
    point_pose.orientation.x = table_path->item(row, 4)->text().toDouble();
    point_pose.orientation.y = table_path->item(row, 5)->text().toDouble();
    point_pose.orientation.z = table_path->item(row, 6)->text().toDouble();
}

void readTableRow(double &x, double &y, double &z, double &qw, double &qx, double &qy, double &qz, QTableWidget *table_path, int row)
{
    x = table_path->item(row, 0)->text().toDouble();
    y = table_path->item(row, 1)->text().toDouble();
    z = table_path->item(row, 2)->text().toDouble();
    qw = table_path->item(row, 3)->text().toDouble();
    qx = table_path->item(row, 4)->text().toDouble();
    qy = table_path->item(row, 5)->text().toDouble();
    qz = table_path->item(row, 6)->text().toDouble();
}

void setTableRow(const VisiblePose &point_pose, QTableWidget *&table_path, int row)
{
    QTableWidgetItem *item0 = new QTableWidgetItem;
    QTableWidgetItem *item1 = new QTableWidgetItem;
    QTableWidgetItem *item2 = new QTableWidgetItem;
    QTableWidgetItem *item3 = new QTableWidgetItem;
    QTableWidgetItem *item4 = new QTableWidgetItem;
    QTableWidgetItem *item5 = new QTableWidgetItem;
    QTableWidgetItem *item6 = new QTableWidgetItem;

    item0->setText(QString::number(point_pose.x, 'c', 2));
    item1->setText(QString::number(point_pose.y, 'c', 2));
    item2->setText(QString::number(point_pose.z, 'c', 2));
    item3->setText(QString::number(point_pose.qw, 'c', 2));
    item4->setText(QString::number(point_pose.qx, 'c', 2));
    item5->setText(QString::number(point_pose.qy, 'c', 2));
    item6->setText(QString::number(point_pose.qz, 'c', 2));

    table_path->setItem(row, 0, item0);
    table_path->setItem(row, 1, item1);
    table_path->setItem(row, 2, item2);
    table_path->setItem(row, 3, item3);
    table_path->setItem(row, 4, item4);
    table_path->setItem(row, 5, item5);
    table_path->setItem(row, 6, item6);
}

void setTableRow(const double &x, const double &y, const double &z, const double &qw, const double &qx, const double &qy, const double &qz, QTableWidget *&table_path, int row)
{
        QTableWidgetItem *item0 = new QTableWidgetItem;
    QTableWidgetItem *item1 = new QTableWidgetItem;
    QTableWidgetItem *item2 = new QTableWidgetItem;
    QTableWidgetItem *item3 = new QTableWidgetItem;
    QTableWidgetItem *item4 = new QTableWidgetItem;
    QTableWidgetItem *item5 = new QTableWidgetItem;
    QTableWidgetItem *item6 = new QTableWidgetItem;

    item0->setText(QString::number(x, 'c', 2));
    item1->setText(QString::number(y, 'c', 2));
    item2->setText(QString::number(z, 'c', 2));
    item3->setText(QString::number(qw, 'c', 2));
    item4->setText(QString::number(qx, 'c', 2));
    item5->setText(QString::number(qy, 'c', 2));
    item6->setText(QString::number(qz, 'c', 2));

    table_path->setItem(row, 0, item0);
    table_path->setItem(row, 1, item1);
    table_path->setItem(row, 2, item2);
    table_path->setItem(row, 3, item3);
    table_path->setItem(row, 4, item4);
    table_path->setItem(row, 5, item5);
    table_path->setItem(row, 6, item6);
}
PathEditPanel::PathEditPanel(QWidget *parent) : QWidget(parent),
                                                ui(new Ui::PathEditPanel)
{
}

PathEditPanel::~PathEditPanel()
{
    delete ui;
}
