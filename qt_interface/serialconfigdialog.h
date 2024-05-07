#ifndef SERIALCONFIGDIALOG_H
#define SERIALCONFIGDIALOG_H

#include <QDialog>
#include <QTimer>
#include <QSerialPort>
#include <QMutex>
#include "rosa_messages.h"
namespace Ui {
class SerialConfigDialog;
}


class SerialConfigDialog : public QDialog
{
    Q_OBJECT
    enum {IDLE, CHECKING, CORRECT, OPENING_ERROR} state{IDLE};
public:
    explicit SerialConfigDialog(QWidget *parent = nullptr);
    ~SerialConfigDialog();

private slots:
     void loop();
     void on_B_check_clicked();
     void update_gui();
     void on_B_Cancel_clicked();

     void on_B_Save_clicked();

     void on_checkBox_toggled(bool checked);

private:
    Ui::SerialConfigDialog *ui;
    QTimer timer;
    QMutex mutex;
    QSerialPort port ;
    QString port_name;
    ROSAmens::MsgReader serial_reader;
    int bauds[5]{9600,19200,38400,57600,115200};
    int counter =0;
    WiFiData w_data{};

    void handle_serial_port();
    void sendMessage(const ROSAmens &m);
    void update_gui_wifi_data();

};

#endif // SERIALCONFIGDIALOG_H
