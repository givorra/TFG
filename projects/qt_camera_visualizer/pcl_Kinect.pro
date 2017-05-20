#-------------------------------------------------
#
# Project created by QtCreator 2014-05-01T14:24:33
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = pcl_visualizer
TEMPLATE = app

SOURCES += src/main.cpp\
        lib/pclKinect.cpp\
        lib/MathMorph.cpp

HEADERS  += include/pclKinect.h\
        include/MathMorph.h

FORMS    += forms/pclKinect.ui
