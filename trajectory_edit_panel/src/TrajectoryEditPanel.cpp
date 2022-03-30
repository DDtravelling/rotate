#include "TrajectoryEditPanel.h"
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_telop_commander::TrajectoryEditPanel, rviz::Panel);
namespace rviz_telop_commander
{

    TrajectoryEditPanel::TrajectoryEditPanel(QWidget *parent)
        : rviz::Panel(parent)
    {
 
    }

}