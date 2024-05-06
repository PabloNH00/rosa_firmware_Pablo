#ifndef SERIALCONFIGDIALOG_H
#define SERIALCONFIGDIALOG_H

#include <QDialog>
#include <QTimer>
namespace Ui {
class SerialConfigDialog;
}

class SerialConfigDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SerialConfigDialog(QWidget *parent = nullptr);
    ~SerialConfigDialog();
private slots:
     void loop();
     void on_B_check_clicked();

private:
    Ui::SerialConfigDialog *ui;
    QTimer timer;
};

#endif // SERIALCONFIGDIALOG_H
