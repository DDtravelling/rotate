#ifndef QT5_COMPOMENT_H
#define QT5_COMPOMENT_H
#include <QApplication>
#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QCommandLinkButton>
#include <QDebug>
#include <QFile>
#include <QFileDialog>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QKeyEvent>
#include <QLCDNumber>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QObject>
#include <QPaintEvent>
#include <QPainter>
#include <QPlainTextEdit>
#include <QProcess>
#include <QPushButton>
#include <QSlider>
#include <QStandardItemModel>
#include <QTabWidget>
#include <QTableView>
#include <QTableWidget>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <QtGui>
#include <QtXml/QDomDocument>
#include <QtXml/QDomElement>
#include <QtXml/QDomNode>
#include <QtXml/QDomNodeList>

class MultiInputDialog : public QDialog
{
    Q_OBJECT
public:
    const int m_GroupCount;
    QLabel **m_Labels;
    QLineEdit **m_LineEdits;
    QPushButton *m_OKButton;
    QPushButton *m_CancelButton;
    QString trajectory_name;
    QString start_point_name;
    QString end_point_name;
    QString describe;
    QVBoxLayout *layout;
    QHBoxLayout *subLayout;
    bool is_ok;

public:
    MultiInputDialog(int count, QWidget *parent) : QDialog(parent), m_GroupCount(count)
    {
        layout = new QVBoxLayout;

        m_Labels = new QLabel *[m_GroupCount];
        m_LineEdits = new QLineEdit *[m_GroupCount];
        //设计界面
        for (int i = 0; i < m_GroupCount; i++)
        {
            subLayout = new QHBoxLayout;

            m_LineEdits[i] = new QLineEdit(this);
            m_Labels[i] = new QLabel(this);
            m_Labels[i]->setText("           ");
            m_Labels[i]->adjustSize();

            layout->addWidget(m_Labels[i]);
            layout->addWidget(m_LineEdits[i]);
            // layout->addLayout(subLayout);
        }

        m_OKButton = new QPushButton(tr("OK"), this);
        m_CancelButton = new QPushButton(tr("Cancel"), this);
        subLayout = new QHBoxLayout;
        subLayout->addStretch();
        subLayout->addWidget(m_OKButton);
        subLayout->addWidget(m_CancelButton);
        layout->addLayout(subLayout);
        setLayout(layout);

        connect(m_OKButton, SIGNAL(clicked()), this, SLOT(accept()));
        connect(m_CancelButton, SIGNAL(clicked()), this, SLOT(reject()));
    }

    ~MultiInputDialog()
    {
        for (int i = 0; i < m_GroupCount; i++)
        {
            delete m_LineEdits[i];
        }
        for (int i = 0; i < m_GroupCount; i++)
        {
            delete m_Labels[i];
        }
        delete m_OKButton;
        delete m_CancelButton;
    }

    void SetLabelTexts(const QStringList &listText)
    {
        for (int i = 0; i < listText.size(); i++)
        {
            if (i >= m_GroupCount)
                break;
            m_Labels[i]->setText(listText.at(i));
            m_Labels[i]->setBaseSize(QSize(200, 50));
        }
    }

    void SetOneLabelText(int index, const QString &text)
    {
        m_Labels[index]->setText(text);
        m_Labels[index]->setBaseSize(QSize(200, 50));
    }

    void SetLabelsWidth(int width)
    {
        for (int i = 0; i < m_GroupCount; i++)
            m_Labels[i]->setFixedWidth(width);
    }

    void SetLineEditRegExp(int index, QRegExp regExp)
    {
        QValidator *validator = new QRegExpValidator(regExp, this);
        m_LineEdits[index]->setValidator(validator);
    }

    QString GetOneText(int index) { return m_LineEdits[index]->text(); }

    QStringList GetAllTexts()
    {
        QStringList list;
        for (int i = 0; i < m_GroupCount; i++)
        {
            list.push_back(m_LineEdits[i]->text());
        }
        return list;
    }

    void accept()
    {
        if (m_GroupCount == 4)
        {
            trajectory_name = m_LineEdits[0]->text();
            start_point_name = m_LineEdits[1]->text();
            end_point_name = m_LineEdits[2]->text();
            describe = m_LineEdits[3]->text();
        }

        if (m_GroupCount == 2)
        {
            trajectory_name = m_LineEdits[0]->text();
            describe = m_LineEdits[1]->text();
        }

        for (int i = 0; i < m_GroupCount; i++)
        {
            m_LineEdits[i]->setText("");
        }
        this->close();
        this->hide();
        is_ok = true;
    }

    void reject()
    {
        for (int i = 0; i < m_GroupCount; i++)
        {
            m_LineEdits[i]->setText("");
        }
        this->close();
        this->hide();
        is_ok = false;
    }
};

#endif