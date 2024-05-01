#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QNetworkInterface>
#include <QNetworkDatagram>
#include <QTextDocument>
#include <QTextBlock>
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow), ip_port(0)
{
    ui->setupUi(this);


    foreach (const QNetworkInterface &netInterface, QNetworkInterface::allInterfaces()) {
        QNetworkInterface::InterfaceFlags flags = netInterface.flags();
        if( (bool)(flags & QNetworkInterface::IsRunning) && !(bool)(flags & QNetworkInterface::IsLoopBack)){
            foreach (const QNetworkAddressEntry &address, netInterface.addressEntries()) {
                if(address.ip().protocol() == QAbstractSocket::IPv4Protocol)
                       ui->CB_NetworkInterfaces->addItem(address.ip().toString());
            }
         }
    }

    connect(&timer, &QTimer::timeout, this, &MainWindow::loop);
    timer.start(50);
}

MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::loop()
{

    static int count=0;
    if(count++>20)count=0;
    if((ip_port)&&(!ui->B_check->isChecked())){
        ip_port->close();
        delete ip_port;
        ip_port=nullptr;
        ui->CB_NetworkInterfaces->setEnabled(true);

    }
    if((!ip_port)&&(ui->B_check->isChecked())){
        ui->CB_NetworkInterfaces->setEnabled(false);
        ip_port=new QUdpSocket(this);
        QHostAddress local(ui->CB_NetworkInterfaces->currentText());
        ip_port->bind(local, ROSA_OUTPUT_UDP_PORT);
        connect(ip_port, &QUdpSocket::readyRead,
                    this, &MainWindow::read_ip_port);
    }
    //regular BROADCAST message
    if((ip_port)&&(count==0)){
        ROSAmens ping(ROSA_SET_MASTER_IP);
        ip_port->writeDatagram((const char *)(ping.data),ping.datagram_size(),QHostAddress::Broadcast,ROSA_INPUT_UDP_PORT);
    }
   ui->heartbeat->loop();
  //fin de test
}
void MainWindow::process_message(ROSAmens &m)
{
    switch(m.id){
        case ROSA_NAME:{
            char name[100]{"HEART BEAT:  "};
            m.read_cstring(name+11,89);
            setText(name);
            ui->heartbeat->reset();
        }break;
        case ROSA_DEBUG_TXT:{
            char text[200]{"DEBUG:  "};
            m.read_cstring(text+7,193);
            setText(text);
        }break;
        case ROSA_ROBOT_DATA:{
            RobotData rd;
            m.read_array<int32_t>(rd.current_velocity,4);
            m.read_array<int32_t>(rd.target_velocity,4);
            m.read_array<int32_t>(rd.encoder_counts,4);
            rd.battery_voltage= m.read<float>();
            update_robot_data(rd);
        }break;
    }
}
void MainWindow::update_robot_data(const RobotData &data){
 ui->cur_vel_1->display(data.current_velocity[0]);
 ui->cur_vel_2->display(data.current_velocity[1]);
 ui->cur_vel_3->display(data.current_velocity[2]);
 ui->cur_vel_4->display(data.current_velocity[3]);

 ui->tar_vel_1->display(data.target_velocity[0]);
 ui->tar_vel_2->display(data.target_velocity[1]);
 ui->tar_vel_3->display(data.target_velocity[2]);
 ui->tar_vel_4->display(data.target_velocity[3]);

 ui->enc_count_1->display(data.encoder_counts[0]);
 ui->enc_count_2->display(data.encoder_counts[1]);
 ui->enc_count_3->display(data.encoder_counts[2]);
 ui->enc_count_4->display(data.encoder_counts[3]);

 ui->battery->display(QString::number(data.battery_voltage, 'f', 1));
}
void MainWindow::read_ip_port()
{

    while (ip_port->hasPendingDatagrams()) {
        QNetworkDatagram datagram = ip_port->receiveDatagram();
        QHostAddress sender=datagram.senderAddress();
        QByteArray data=datagram.data();
        for (int i = 0; i < data.size(); ++i) {
            if(udp_reader.add_uchar(data[i])){
                auto &&msg=udp_reader.getMessage();
                if(msg.size)process_message(msg);
                if((msg.size)&&(msg.id==ROSA_NAME))info(QString("Robot conected at IP ")+sender.toString());
            }
      }
  }
}
void MainWindow::setText(char *text){
    ui->TE_info->append(QString((char *)text));
    ui->TE_info->moveCursor(QTextCursor::End);
}
void MainWindow::info(const QString &mens)
{
    ui->statusbar->showMessage(mens,5000);
    update();
}
