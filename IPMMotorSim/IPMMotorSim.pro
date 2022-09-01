#-------------------------------------------------
#
# Project created by QtCreator 2022-08-20T13:18:00
#
#-------------------------------------------------

QT += core gui
QT += charts

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = IPMMotorSim
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS
DEFINES += CTRL_FOC=1
DEFINES += CTRL_SINE=0
DEFINES += CONTROL=1

DEFINES += STM32F1

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++11

INCLUDEPATH += $$PWD/stm32-sine/include
INCLUDEPATH += $$PWD/stm32-sine/libopencm3/include
INCLUDEPATH += $$PWD/stm32-sine/libopeninv/include

SOURCES += \
        main.cpp \
        mainwindow.cpp \
    chart.cpp \
    chartview.cpp \
    datagraph.cpp \
    motormodel.cpp \
    stm32-sine/libopeninv/src/params.cpp \
    stm32-sine/libopeninv/src/picontroller.cpp \
    stm32-sine/libopeninv/src/sine_core.cpp \
    stm32-sine/src/pwmgeneration-foc.cpp \
    stm32-sine/libopeninv/src/my_string.c \
    stm32-sine/libopeninv/src/errormessage.cpp \
    stm32-sine/libopeninv/src/foc.cpp \
    teststubs.c \
    cpp_teststubs.cpp \
    stm32-sine/src/pwmgeneration.cpp

HEADERS += \
        mainwindow.h \
    chart.h \
    chartview.h \
    datagraph.h \
    motormodel.h \
    stm32-sine/include/pwmgeneration.h \
    teststubs.h

FORMS += \
        mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
