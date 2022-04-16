#ifndef PATHEDITPANEL_H
#define PATHEDITPANEL_H

#include <QWidget>

namespace Ui {
class PathEditPanel;
}

class PathEditPanel : public QWidget
{
    Q_OBJECT

public:
    explicit PathEditPanel(QWidget *parent = 0);
    ~PathEditPanel();


    Ui::PathEditPanel *ui;
};

#endif // PATHEDITPANEL_H
