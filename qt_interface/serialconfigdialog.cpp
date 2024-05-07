#include "serialconfigdialog.h"
#include "ui_serialconfigdialog.h"
#include "QtSerialPort/QSerialPortInfo"
#include "QDialogButtonBox"
#include "QDebug"
#include "QMessageBox"
SerialConfigDialog::SerialConfigDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SerialConfigDialog)
{
    ui->setupUi(this);
    foreach (const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts()){
        ui->CB_serialPorts->addItem(serialPortInfo.portName());
        }
    connect(&timer, &QTimer::timeout, this, &SerialConfigDialog::loop);
    for(auto b:bauds)ui->CB_baudRates->addItem(QString::number(b));
    ui->CB_baudRates->setCurrentIndex(2);


    QString ipRange = "(?:[0-1]?[0-9]?[0-9]|2[0-4][0-9]|25[0-5])";
    QRegExp ipRegex ("^" + ipRange
                     + "\\." + ipRange
                     + "\\." + ipRange
                     + "\\." + ipRange + "$");

    QRegExpValidator *ipValidator = new QRegExpValidator(ipRegex, this);
    ui->edit_ip->setValidator(ipValidator);
    QRegExpValidator *gatewayValidator = new QRegExpValidator(ipRegex, this);
    ui->edit_ip->setValidator(gatewayValidator);
    QRegExpValidator *maskValidator = new QRegExpValidator(ipRegex, this);
    ui->edit_ip->setValidator(maskValidator);

    timer.start(50);
}

SerialConfigDialog::~SerialConfigDialog()
{
    delete ui;
}
void SerialConfigDialog::loop()
{
    handle_serial_port();
    if(state==CHECKING){
        if(counter<100)counter++;
    }
    update_gui();
}
void SerialConfigDialog::handle_serial_port()
{
    if(!port.isOpen())return;
    QByteArray data = port.readAll();
    for (int i = 0; i < data.size(); ++i) {
        if(serial_reader.add_uchar(data[i])){
            auto m=serial_reader.getMessage();
            if(m.id==ROSA_WIFI_INFO){
                m.read_array<uint8_t>(w_data.ip,4);
                m.read_array<uint8_t>(w_data.gateway,4);
                m.read_array<uint8_t>(w_data.mask,4);
                m.read_cstring(w_data.ssid,50);
                m.read_cstring(w_data.key,50);
                update_gui_wifi_data();
                if(state==CHECKING)state=CORRECT;
            }
            if(m.id==ROSA_WIFI_CONFIGURED){
                QMessageBox msgBox;
                auto val=m.read<uint8_t>();
                if(val)msgBox.setText("WiFi Info Correctly saved");
                else msgBox.setText("Something went wrong when saving WiFi Info");
                msgBox.exec();
            }

        }
    }
}
void SerialConfigDialog::update_gui_wifi_data()
{

    ui->edit_wifi->setText(w_data.ssid);
    ui->edit_passwd->setText(w_data.key);
    ui->edit_ip->setText(QString("%1.%2.%3.%4").arg(w_data.ip[0]).arg(w_data.ip[1]).arg(w_data.ip[2]).arg(w_data.ip[3]));
    ui->edit_gateway->setText(QString("%1.%2.%3.%4").arg(w_data.gateway[0]).arg(w_data.gateway[1]).arg(w_data.gateway[2]).arg(w_data.gateway[3]));
    ui->edit_mask->setText(QString("%1.%2.%3.%4").arg(w_data.mask[0]).arg(w_data.mask[1]).arg(w_data.mask[2]).arg(w_data.mask[3]));
}
void SerialConfigDialog::on_B_check_clicked()
{
    port_name=ui->CB_serialPorts->currentText();

    port.setPortName(port_name);
    port.setBaudRate(this->bauds[ui->CB_baudRates->currentIndex()]);
    if(port.open(QIODevice::ReadWrite)){
      state=CHECKING;
      counter=0;
      ui->B_check->setEnabled(false);
      sendMessage(ROSAmens(ROSA_GET_WIFI_CONFIG));
      return;
    }
    counter = 50;
    ui->CB_serialPorts->removeItem(ui->CB_serialPorts->currentIndex());



}
void SerialConfigDialog::sendMessage(const ROSAmens &m){
    QByteArray mens((const char *)(m.data),m.datagram_size());
    if(port.isOpen()){
        port.write(mens);
        port.flush();
    }
}
void SerialConfigDialog::update_gui()
{
    QString text;
    switch(state){
        case IDLE:
          ui->B_Save->setEnabled(false);
          ui->B_check->setEnabled(true);
          ui->CB_baudRates->setEnabled(true);
          ui->CB_serialPorts->setEnabled(true);
          if(counter){
              --counter;
              text="No robot present at port "+port_name;
          }else text="Select a serial port and check";
        break;
        case CHECKING:
          ui->B_check->setEnabled(false);
          ui->CB_baudRates->setEnabled(false);
          ui->CB_serialPorts->setEnabled(false);
          text="Cheking if there is a robot at " +
                port_name + QString().asprintf("-->(%2d%%)",counter);
          if(counter==100){
              state=IDLE;
              port.close();
          }

        break;
        case CORRECT:
          ui->B_Save->setEnabled(true);
          ui->B_check->setEnabled(false);
          ui->CB_baudRates->setEnabled(false);
          ui->CB_serialPorts->setEnabled(false);
          text="Robot connected at port "+port_name;
        break;
        case OPENING_ERROR:
            text = "Unable to open the serial port "+port_name;
            if(!--counter)state = IDLE;

        break;
    default:
        state = IDLE;
    }
    ui->info->setText(text);
}

void SerialConfigDialog::on_B_Cancel_clicked()
{
this->close();
}

#ifdef _WIN32
    #include <Ws2tcpip.h>
#else
    #include <sys/socket.h>
    #include <sys/types.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
#endif

//#include <cstring>
void SerialConfigDialog::on_B_Save_clicked()
{

    inet_pton(AF_INET,ui->edit_ip->text().toStdString().c_str(),w_data.ip);
    inet_pton(AF_INET,ui->edit_gateway->text().toStdString().c_str(),w_data.gateway);
    inet_pton(AF_INET,ui->edit_mask->text().toStdString().c_str(),w_data.mask);
    auto ssid = ui->edit_wifi->text().toStdString();
    for(auto &p:w_data.ssid)p=0;
    std::copy(ssid.begin(), ssid.end(), w_data.ssid);
    for(auto &p:w_data.key)p=0;
    auto key = ui->edit_passwd->text().toStdString();
    std::copy(key.begin(), key.end(), w_data.key);

    sendMessage(info_wifi_message(ROSA_SET_WIFI_INFO,w_data));
}

void SerialConfigDialog::on_checkBox_toggled(bool checked)
{
    sendMessage(ROSAmens(ROSA_ENABLE_ROBOCLAWS,!checked));
}
