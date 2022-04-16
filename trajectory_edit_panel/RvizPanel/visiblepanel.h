#ifndef VISIBLEPANEL_H
#define VISIBLEPANEL_H

#include <QWidget>

namespace Ui {
class visiblePanel;
}

class visiblePanel : public QWidget
{
    Q_OBJECT

public:
    explicit visiblePanel(QWidget *parent = 0);
    ~visiblePanel();


    Ui::visiblePanel *ui;
};

#endif // VISIBLEPANEL_H
