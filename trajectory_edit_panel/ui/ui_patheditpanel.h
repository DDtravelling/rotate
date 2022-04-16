/********************************************************************************
** Form generated from reading UI file 'patheditpanel.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PATHEDITPANEL_H
#define UI_PATHEDITPANEL_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_PathEditPanel
{
public:
    QGroupBox *groupbox_path;
    QTableWidget *table_path;
    QTabWidget *tabWidget;
    QWidget *tab;
    QComboBox *combox_axies;
    QLabel *label;
    QLineEdit *line_rotate;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *label_rotate_value;
    QLabel *label_5;
    QPushButton *button_rotate;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
    QPushButton *pushButton_3;
    QPushButton *pushButton_4;
    QPushButton *pushButton_5;
    QPushButton *pushButton_6;
    QLabel *label_6;
    QLabel *label_move_value;
    QLabel *label_7;
    QLabel *label_8;
    QLineEdit *line_move_step;
    QWidget *tab_2;

    void hide()
    {
        tabWidget->hide();
        groupbox_path->hide();
    }

    void show()
    {
        tabWidget->show();
        groupbox_path->show();
    }
    void setupUi(QWidget *PathEditPanel)
    {
        if (PathEditPanel->objectName().isEmpty())
            PathEditPanel->setObjectName(QStringLiteral("PathEditPanel"));
        PathEditPanel->resize(1006, 770);
        groupbox_path = new QGroupBox(PathEditPanel);
        groupbox_path->setObjectName(QStringLiteral("groupbox_path"));
        groupbox_path->setGeometry(QRect(110, 10, 751, 481));
        table_path = new QTableWidget(groupbox_path);
        table_path->setObjectName(QStringLiteral("table_path"));
        table_path->setGeometry(QRect(10, 30, 721, 441));
        tabWidget = new QTabWidget(PathEditPanel);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tabWidget->setGeometry(QRect(30, 550, 861, 201));
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        combox_axies = new QComboBox(tab);
        combox_axies->setObjectName(QStringLiteral("combox_axies"));
        combox_axies->setGeometry(QRect(90, 20, 86, 31));
        label = new QLabel(tab);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(0, 20, 71, 31));
        QFont font;
        font.setPointSize(12);
        label->setFont(font);
        line_rotate = new QLineEdit(tab);
        line_rotate->setObjectName(QStringLiteral("line_rotate"));
        line_rotate->setGeometry(QRect(90, 70, 61, 21));
        label_2 = new QLabel(tab);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(160, 60, 31, 31));
        label_2->setFont(font);
        label_3 = new QLabel(tab);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(0, 60, 71, 31));
        label_3->setFont(font);
        label_4 = new QLabel(tab);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(0, 100, 71, 31));
        label_4->setFont(font);
        label_rotate_value = new QLabel(tab);
        label_rotate_value->setObjectName(QStringLiteral("label_rotate_value"));
        label_rotate_value->setGeometry(QRect(90, 100, 61, 31));
        label_rotate_value->setFont(font);
        label_5 = new QLabel(tab);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(160, 100, 31, 31));
        label_5->setFont(font);
        button_rotate = new QPushButton(tab);
        button_rotate->setObjectName(QStringLiteral("button_rotate"));
        button_rotate->setGeometry(QRect(40, 140, 89, 25));
        pushButton = new QPushButton(tab);
        pushButton->setObjectName(QStringLiteral("pushButton"));
        pushButton->setGeometry(QRect(360, 20, 51, 31));
        pushButton_2 = new QPushButton(tab);
        pushButton_2->setObjectName(QStringLiteral("pushButton_2"));
        pushButton_2->setGeometry(QRect(360, 130, 51, 31));
        pushButton_3 = new QPushButton(tab);
        pushButton_3->setObjectName(QStringLiteral("pushButton_3"));
        pushButton_3->setGeometry(QRect(290, 70, 51, 31));
        pushButton_4 = new QPushButton(tab);
        pushButton_4->setObjectName(QStringLiteral("pushButton_4"));
        pushButton_4->setGeometry(QRect(430, 70, 51, 31));
        pushButton_5 = new QPushButton(tab);
        pushButton_5->setObjectName(QStringLiteral("pushButton_5"));
        pushButton_5->setGeometry(QRect(530, 20, 51, 31));
        pushButton_6 = new QPushButton(tab);
        pushButton_6->setObjectName(QStringLiteral("pushButton_6"));
        pushButton_6->setGeometry(QRect(530, 130, 51, 31));
        label_6 = new QLabel(tab);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(620, 10, 71, 31));
        label_6->setFont(font);
        label_move_value = new QLabel(tab);
        label_move_value->setObjectName(QStringLiteral("label_move_value"));
        label_move_value->setGeometry(QRect(700, 20, 61, 81));
        label_move_value->setFont(font);
        label_7 = new QLabel(tab);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(620, 130, 71, 31));
        label_7->setFont(font);
        label_8 = new QLabel(tab);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setGeometry(QRect(770, 130, 31, 31));
        label_8->setFont(font);
        line_move_step = new QLineEdit(tab);
        line_move_step->setObjectName(QStringLiteral("line_move_step"));
        line_move_step->setGeometry(QRect(700, 140, 61, 21));
        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QStringLiteral("tab_2"));
        tabWidget->addTab(tab_2, QString());

        retranslateUi(PathEditPanel);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(PathEditPanel);
    } // setupUi

    void retranslateUi(QWidget *PathEditPanel)
    {
        PathEditPanel->setWindowTitle(QApplication::translate("PathEditPanel", "Form", Q_NULLPTR));
        groupbox_path->setTitle(QApplication::translate("PathEditPanel", "\350\246\206\347\233\226\350\275\250\350\277\271\357\274\232", Q_NULLPTR));
        combox_axies->clear();
        combox_axies->insertItems(0, QStringList()
         << QApplication::translate("PathEditPanel", "\350\207\252\350\272\253X\350\275\264", Q_NULLPTR)
         << QApplication::translate("PathEditPanel", "\350\207\252\350\272\253Y\350\275\264", Q_NULLPTR)
         << QApplication::translate("PathEditPanel", "\350\207\252\350\272\253Z\350\275\264", Q_NULLPTR)
         << QApplication::translate("PathEditPanel", "\345\205\250\345\261\200X\350\275\264", Q_NULLPTR)
         << QApplication::translate("PathEditPanel", "\345\205\250\345\261\200Y\350\275\264", Q_NULLPTR)
         << QApplication::translate("PathEditPanel", "\345\205\250\345\261\200Z\350\275\264", Q_NULLPTR)
        );
        label->setText(QApplication::translate("PathEditPanel", "\346\227\213\350\275\254\350\275\264    \357\274\232", Q_NULLPTR));
        label_2->setText(QApplication::translate("PathEditPanel", "\345\272\246", Q_NULLPTR));
        label_3->setText(QApplication::translate("PathEditPanel", "\346\227\213\350\275\254\350\247\222\345\272\246\357\274\232", Q_NULLPTR));
        label_4->setText(QApplication::translate("PathEditPanel", "\346\227\213\350\275\254\345\242\236\351\207\217\357\274\232", Q_NULLPTR));
        label_rotate_value->setText(QString());
        label_5->setText(QApplication::translate("PathEditPanel", "\345\272\246", Q_NULLPTR));
        button_rotate->setText(QApplication::translate("PathEditPanel", "\346\227\213\350\275\254", Q_NULLPTR));
        pushButton->setText(QApplication::translate("PathEditPanel", "\345\220\216\351\200\200", Q_NULLPTR));
        pushButton_2->setText(QApplication::translate("PathEditPanel", "\345\211\215\350\277\233", Q_NULLPTR));
        pushButton_3->setText(QApplication::translate("PathEditPanel", "\345\267\246\347\247\273", Q_NULLPTR));
        pushButton_4->setText(QApplication::translate("PathEditPanel", "\345\217\263\347\247\273", Q_NULLPTR));
        pushButton_5->setText(QApplication::translate("PathEditPanel", "\344\270\212\347\247\273", Q_NULLPTR));
        pushButton_6->setText(QApplication::translate("PathEditPanel", "\344\270\213\347\247\273", Q_NULLPTR));
        label_6->setText(QApplication::translate("PathEditPanel", "\347\247\273\345\212\250\345\242\236\351\207\217\357\274\232", Q_NULLPTR));
        label_move_value->setText(QString());
        label_7->setText(QApplication::translate("PathEditPanel", "\347\247\273\345\212\250\346\255\245\351\225\277\357\274\232", Q_NULLPTR));
        label_8->setText(QApplication::translate("PathEditPanel", "\347\261\263", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("PathEditPanel", "\346\227\213\350\275\254\345\271\263\347\247\273\350\275\250\350\277\271", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("PathEditPanel", "\345\210\240\351\231\244\350\275\250\350\277\271", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class PathEditPanel: public Ui_PathEditPanel {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PATHEDITPANEL_H
