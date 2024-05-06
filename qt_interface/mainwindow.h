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
    void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;

    ROSAmens::MsgReader udp_reader;
    QUdpSocket * ip_port;
    QTimer timer;
    void read_ip_port();
    void setText(char *text);
    void process_message(ROSAmens &m);
    void info(const QString &mens);
    void update_robot_data(const RobotData &data);
};
#endif // MAINWINDOW_H
