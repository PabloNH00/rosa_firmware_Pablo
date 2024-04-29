#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QNetworkInterface>
#include <QNetworkDatagram>

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

    if(!ip_port){
        ip_port=new QUdpSocket(this);
        QHostAddress local(ui->CB_NetworkInterfaces->currentText());
        ip_port->bind(local, ROSA_OUTPUT_UDP_PORT);
        connect(ip_port, &QUdpSocket::readyRead,
                    this, &MainWindow::read_ip_port);
    }
    //regular BROADCAST message
    if(count==0){
        ROSAmens ping(ROSA_SET_MASTER_IP);
        ip_port->writeDatagram((const char *)(ping.data),ping.datagram_size(),QHostAddress::Broadcast,ROSA_INPUT_UDP_PORT);
    }

  //fin de test
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
                if((msg.size)&&(msg.id==ROSA_NAME)){
                   /*char name[100]="";
                   rm_getString(msg.info,name,99);
                   QString qname(name);
                   info(QString("Robot ")+qname+QString(" detected at IP ")+sender.toString());
                   auto module=ModulesHandler::getWithName(qname);
                   module->ip=sender;
                   if(!module->tab){
                      module->tab=new RobominersModule;
                      ui->tabWidget->addTab((QWidget *)(module->tab),qname);
                      module->tab->setModule(module);
                      module->reset_wifi_watchdog();
                   }
                   updateTable();*/

                }
            }
      }
  }
}
