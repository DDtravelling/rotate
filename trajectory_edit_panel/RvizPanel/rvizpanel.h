#ifndef RVIZPANEL_H
#define RVIZPANEL_H

#include <QWidget>
#include "patheditpanel.h"
#include "visiblepanel.h"

namespace Ui {
class RvizPanel;
}

class RvizPanel : public QWidget
{
    Q_OBJECT

public:
    explicit RvizPanel(QWidget *parent = 0);
    ~RvizPanel();

    PathEditPanel path_edit_panel;
    visiblePanel visible_panel;

private slots:
    void on_comboBox_currentIndexChanged(int index);

private:
    Ui::RvizPanel *ui;
};

#endif // RVIZPANEL_H
