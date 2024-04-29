#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QUdpSocket>
#include <QTimer>
#include "rosa_messages.h"
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
private slots:

    void loop();
private:
    Ui::MainWindow *ui;
    ROSAmens::MsgReader udp_reader;
    QUdpSocket * ip_port;
    QTimer timer;
    void read_ip_port();
};
#endif // MAINWINDOW_H
