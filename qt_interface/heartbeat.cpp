#include "heartbeat.h"
#include <QPainter>
HeartBeat::HeartBeat(QWidget *parent) : QWidget(parent)
{

}

void HeartBeat::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event)

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    //painter.setRenderHint(QPainter::Antialiasing);


    m_knopBounds.setWidth(width() * factor);
    m_knopBounds.setHeight(height()* factor);
     m_knopBounds.moveCenter(QPointF(m_bounds.center().x(), m_bounds.center().y() ));

    // draw background
    QRadialGradient gradient(m_bounds.center(), m_bounds.width()/2, m_bounds.center());

    gradient = QRadialGradient(m_knopBounds.center(), m_knopBounds.width()/2, m_knopBounds.center());
    gradient.setColorAt(0, QColor(200,40,20));//red
    gradient.setColorAt(1, QColor(100,0,0)); //darkred
    gradient.setFocalRadius(m_knopBounds.width()*0.2);
    gradient.setCenterRadius(m_knopBounds.width()*0.5);

    painter.setPen(QPen(QBrush(Qt::darkGray), m_bounds.width()*0.005));
    painter.setBrush(QBrush(gradient));
    painter.drawEllipse(m_knopBounds);
}
void HeartBeat::resizeEvent(QResizeEvent *event)
{
    Q_UNUSED(event)

    float a = qMin(width(), height());

    QPointF topleft(0,0);


    m_bounds = QRectF(topleft, QSize(width(), height()));


    m_knopBounds.setWidth(width() * factor);
    m_knopBounds.setHeight(height()* factor);


    m_knopBounds.moveCenter(QPointF(m_bounds.center().x(), m_bounds.center().y() ));
}
