#include "visiblepanel.h"
#include "ui_visiblepanel.h"

visiblePanel::visiblePanel(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::visiblePanel)
{

}

visiblePanel::~visiblePanel()
{
    delete ui;
}
