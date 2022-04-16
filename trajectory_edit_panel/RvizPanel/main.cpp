#include "rvizpanel.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    RvizPanel w;
    w.show();

    return a.exec();
}
