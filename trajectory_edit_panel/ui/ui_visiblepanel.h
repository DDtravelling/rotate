/********************************************************************************
** Form generated from reading UI file 'visiblepanel.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_VISIBLEPANEL_H
#define UI_VISIBLEPANEL_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_visiblePanel
{
public:
    QGroupBox *groupbox_path_set;
    QLabel *label;
    QComboBox *combox_path_mdl;

    void show()
    {
        groupbox_path_set->show();
    }

    void hide()
    {
        groupbox_path_set->hide();
    }
    void setupUi(QWidget *visiblePanel)
    {
        if (visiblePanel->objectName().isEmpty())
            visiblePanel->setObjectName(QStringLiteral("visiblePanel"));
        visiblePanel->resize(1016, 752);
        groupbox_path_set = new QGroupBox(visiblePanel);
        groupbox_path_set->setObjectName(QStringLiteral("groupbox_path_set"));
        groupbox_path_set->setGeometry(QRect(20, 130, 281, 211));
        label = new QLabel(groupbox_path_set);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(10, 40, 111, 21));
        QFont font;
        font.setPointSize(12);
        label->setFont(font);
        combox_path_mdl = new QComboBox(groupbox_path_set);
        combox_path_mdl->setObjectName(QStringLiteral("combox_path_mdl"));
        combox_path_mdl->setGeometry(QRect(130, 40, 86, 25));

        retranslateUi(visiblePanel);

        QMetaObject::connectSlotsByName(visiblePanel);
    } // setupUi

    void retranslateUi(QWidget *visiblePanel)
    {
        visiblePanel->setWindowTitle(QApplication::translate("visiblePanel", "Form", Q_NULLPTR));
        groupbox_path_set->setTitle(QApplication::translate("visiblePanel", "\350\275\250\350\277\271\350\256\276\347\275\256", Q_NULLPTR));
        label->setText(QApplication::translate("visiblePanel", "\346\230\276\347\244\272\346\250\241\345\274\217\350\256\276\347\275\256\357\274\232", Q_NULLPTR));
        combox_path_mdl->clear();
        combox_path_mdl->insertItems(0, QStringList()
         << QApplication::translate("visiblePanel", "\344\275\215\345\247\277\346\250\241\345\274\217", Q_NULLPTR)
         << QApplication::translate("visiblePanel", "\346\233\262\347\272\277\346\250\241\345\274\217", Q_NULLPTR)
        );
    } // retranslateUi

};

namespace Ui {
    class visiblePanel: public Ui_visiblePanel {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_VISIBLEPANEL_H
