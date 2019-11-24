#-------------------------------------------------
#
# Project created by QtCreator 2018-11-20T14:26:06
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = dul_cam5640
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += main.cpp\
        mainwindow.cpp \
    stitch.cpp \
    stitch_orb_surf.cpp

HEADERS  += mainwindow.h \
    stitch.h \
    stitch_orb_surf.h

FORMS    += mainwindow.ui

INCLUDEPATH += /usr/local/opencvlib_gnueabihf/include \
                /usr/local/opencvlib_gnueabihf/include/opencv \
                /usr/local/opencvlib_gnueabihf/include/opencv2
LIBS += /usr/local/opencvlib_gnueabihf/lib/libopencv_aruco.so\
/usr/local/opencvlib_gnueabihf/lib/libopencv_phase_unwrapping.so\
/usr/local/opencvlib_gnueabihf/lib/libopencv_bgsegm.so          \
/usr/local/opencvlib_gnueabihf/lib/libopencv_photo.so\
/usr/local/opencvlib_gnueabihf/lib/libopencv_bioinspired.so      \
/usr/local/opencvlib_gnueabihf/lib/libopencv_plot.so\
/usr/local/opencvlib_gnueabihf/lib/libopencv_calib3d.so    \
/usr/local/opencvlib_gnueabihf/lib/libopencv_reg.so\
/usr/local/opencvlib_gnueabihf/lib/libopencv_ccalib.so    \
/usr/local/opencvlib_gnueabihf/lib/libopencv_rgbd.so\
/usr/local/opencvlib_gnueabihf/lib/libopencv_core.so  \
/usr/local/opencvlib_gnueabihf/lib/libopencv_saliency.so\
/usr/local/opencvlib_gnueabihf/lib/libopencv_datasets.so\
/usr/local/opencvlib_gnueabihf/lib/libopencv_shape.so\
/usr/local/opencvlib_gnueabihf/lib/libopencv_dnn.so  \
/usr/local/opencvlib_gnueabihf/lib/libopencv_stereo.so\
/usr/local/opencvlib_gnueabihf/lib/libopencv_dpm.so   \
/usr/local/opencvlib_gnueabihf/lib/libopencv_stitching.so\
/usr/local/opencvlib_gnueabihf/lib/libopencv_face.so    \
/usr/local/opencvlib_gnueabihf/lib/libopencv_structured_light.so\
/usr/local/opencvlib_gnueabihf/lib/libopencv_features2d.so \
/usr/local/opencvlib_gnueabihf/lib/libopencv_superres.so\
/usr/local/opencvlib_gnueabihf/lib/libopencv_flann.so   \
/usr/local/opencvlib_gnueabihf/lib/libopencv_surface_matching.so\
/usr/local/opencvlib_gnueabihf/lib/libopencv_text.so\
/usr/local/opencvlib_gnueabihf/lib/libopencv_fuzzy.so    \
/usr/local/opencvlib_gnueabihf/lib/libopencv_tracking.so\
/usr/local/opencvlib_gnueabihf/lib/libopencv_highgui.so \
/usr/local/opencvlib_gnueabihf/lib/libopencv_video.so\
/usr/local/opencvlib_gnueabihf/lib/libopencv_imgcodecs.so   \
/usr/local/opencvlib_gnueabihf/lib/libopencv_videostab.so\
/usr/local/opencvlib_gnueabihf/lib/libopencv_img_hash.so \
/usr/local/opencvlib_gnueabihf/lib/libopencv_imgproc.so \
/usr/local/opencvlib_gnueabihf/lib/libopencv_xfeatures2d.so\
/usr/local/opencvlib_gnueabihf/lib/libopencv_line_descriptor.so \
/usr/local/opencvlib_gnueabihf/lib/libopencv_ximgproc.so\
/usr/local/opencvlib_gnueabihf/lib/libopencv_ml.so        \
/usr/local/opencvlib_gnueabihf/lib/libopencv_xobjdetect.so\
/usr/local/opencvlib_gnueabihf/lib/libopencv_objdetect.so \
/usr/local/opencvlib_gnueabihf/lib/libopencv_xphoto.so\
/usr/local/opencvlib_gnueabihf/lib/libopencv_optflow.so
