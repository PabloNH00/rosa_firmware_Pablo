#include "serialconfigdialog.h"
#include "ui_serialconfigdialog.h"
#include "QtSerialPort/QSerialPortInfo"

SerialConfigDialog::SerialConfigDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SerialConfigDialog)
{
    ui->setupUi(this);
    foreach (const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts()){
        ui->CB_serialPorts->addItem(serialPortInfo.portName());
        }
    connect(&timer, &QTimer::timeout, this, &SerialConfigDialog::loop);
    timer.start(50);
}

SerialConfigDialog::~SerialConfigDialog()
{
    delete ui;
}
void SerialConfigDialog::loop()
{
    static int i=0;
//ui->info->setText(QString::number(i++));
}

void SerialConfigDialog::on_B_check_clicked()
{
    ui->info->setText("CHECKING FOR ROBOT AT PORT "+ui->CB_serialPorts->currentText());
}
