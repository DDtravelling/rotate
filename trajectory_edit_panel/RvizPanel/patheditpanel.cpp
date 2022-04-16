#include "patheditpanel.h"
#include "ui_patheditpanel.h"

PathEditPanel::PathEditPanel(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PathEditPanel)
{

}

PathEditPanel::~PathEditPanel()
{
    delete ui;
}
