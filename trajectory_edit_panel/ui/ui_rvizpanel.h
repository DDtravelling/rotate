/********************************************************************************
** Form generated from reading UI file 'rvizpanel.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_RVIZPANEL_H
#define UI_RVIZPANEL_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_RvizPanel
{
public:
    QComboBox *comboBox;

    void setupUi(QWidget *RvizPanel)
    {
        if (RvizPanel->objectName().isEmpty())
            RvizPanel->setObjectName(QStringLiteral("RvizPanel"));
        RvizPanel->resize(1099, 838);
        comboBox = new QComboBox(RvizPanel);
        comboBox->setObjectName(QStringLiteral("comboBox"));
        comboBox->setGeometry(QRect(0, 0, 101, 25));

        retranslateUi(RvizPanel);

        QMetaObject::connectSlotsByName(RvizPanel);
    } // setupUi

    void retranslateUi(QWidget *RvizPanel)
    {
        RvizPanel->setWindowTitle(QApplication::translate("RvizPanel", "RvizPanel", Q_NULLPTR));
        comboBox->clear();
        comboBox->insertItems(0, QStringList()
         << QApplication::translate("RvizPanel", "\345\216\237\345\247\213\351\235\242\346\235\277", Q_NULLPTR)
         << QApplication::translate("RvizPanel", "\350\275\250\350\277\271\347\224\237\346\210\220\351\235\242\346\235\277", Q_NULLPTR)
         << QApplication::translate("RvizPanel", "\345\217\257\350\247\206\345\214\226\350\260\203\346\225\264\351\235\242\346\235\277", Q_NULLPTR)
        );
    } // retranslateUi

};

namespace Ui {
    class RvizPanel: public Ui_RvizPanel {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_RVIZPANEL_H
