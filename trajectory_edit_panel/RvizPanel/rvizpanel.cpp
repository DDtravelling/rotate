#include "rvizpanel.h"
#include "ui_rvizpanel.h"
#include "ui_patheditpanel.h"
#include "ui_visiblepanel.h"

RvizPanel::RvizPanel(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::RvizPanel)
{
    ui->setupUi(this);

    path_edit_panel.ui->setupUi(this);
    visible_panel.ui->setupUi(this);
    path_edit_panel.ui->hide();
    visible_panel.ui->hide();

}

RvizPanel::~RvizPanel()
{
    delete ui;
}

void RvizPanel::on_comboBox_currentIndexChanged(int index)
{

    if(index == 0)
    {
        path_edit_panel.ui->hide();
        visible_panel.ui->hide();
    }

    else if(index == 1)
    {
        visible_panel.ui->hide();
        path_edit_panel.ui->show();
    }

    else if(index == 2)
    {
        path_edit_panel.ui->hide();
        visible_panel.ui->show();
    }
}
