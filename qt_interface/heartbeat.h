#ifndef HEARTBEAT_H
#define HEARTBEAT_H

#include <QWidget>

class HeartBeat : public QWidget
{
    Q_OBJECT
public:
    explicit HeartBeat(QWidget *parent = nullptr);
    void loop(double vel=0.01){if(factor>vel)factor-=vel;update();};
    void reset(){factor=1.0F;update();};

signals:
private:
    QRectF m_bounds;
    QRectF m_knopBounds;
    qreal factor{0.3};
    virtual void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;
};

#endif // HEARTBEAT_H
