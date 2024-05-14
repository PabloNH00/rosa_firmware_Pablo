QT       += core gui network serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
greaterThan(QT_MAJOR_VERSION, 5): QT += core5compat
lessThan(QT_MAJOR_VERSION, 6):QMAKE_CXXFLAGS += "/bigobj"
CONFIG += c++17


# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    heartbeat.cpp \
    main.cpp \
    mainwindow.cpp \
    serialconfigdialog.cpp

HEADERS += \
    heartbeat.h \
    mainwindow.h \
    ../shared_src/rosa_messages.h \
    ../shared_src/spslib.h \
    serialconfigdialog.h

FORMS += \
    mainwindow.ui \
    serialconfigdialog.ui


win32 {
        LIBS += Ws2_32.lib
    }
INCLUDEPATH += "../shared_src"

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
