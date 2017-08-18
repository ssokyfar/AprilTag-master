TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

QMAKE_CXXFLAGS += -std=c++0x
QMAKE_CFLAGS += -std=c99

INCLUDEPATH += /usr/local/include \
               /usr/local/include/opencv \
               /usr/local/include/opencv2

LIBS +=-L/usr/local/lib -lopencv_cudabgsegm -lopencv_cudaobjdetect -lopencv_cudastereo -lopencv_shape -lopencv_stitching -lopencv_cudafeatures2d -lopencv_superres -lopencv_cudacodec -lopencv_videostab -lopencv_cudaoptflow -lopencv_cudalegacy -lopencv_photo -lopencv_cudawarping -lopencv_cudaimgproc -lopencv_cudafilters -lopencv_cudaarithm -lopencv_aruco -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_dpm -lopencv_face -lopencv_freetype -lopencv_fuzzy -lopencv_img_hash -lopencv_line_descriptor -lopencv_optflow -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_stereo -lopencv_structured_light -lopencv_viz -lopencv_phase_unwrapping -lopencv_surface_matching -lopencv_tracking -lopencv_datasets -lopencv_text -lopencv_dnn -lopencv_video -lopencv_plot -lopencv_ml -lopencv_ximgproc -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_flann -lopencv_xobjdetect -lopencv_imgcodecs -lopencv_objdetect -lopencv_xphoto -lopencv_imgproc -lopencv_core -lopencv_cudev

LIBS += /usr/lib/x86_64-linux-gnu/libpthread.so \

HEADERS += \
    tag36h11.h \
    tag36h10.h \
    tag36artoolkit.h \
    tag25h9.h \
    tag25h7.h \
    tag16h5.h \
    apriltag_math.h \
    apriltag.h \
    common/zmaxheap.h \
    common/zhash.h \
    common/zarray.h \
    common/workerpool.h \
    common/unionfind.h \
    common/time_util.h \
    common/timeprofile.h \
    common/thash_impl.h \
    common/svd22.h \
    common/string_util.h \
    common/postscript_utils.h \
    common/pnm.h \
    common/pjpeg.h \
    common/pam.h \
    common/math_util.h \
    common/matd.h \
    common/image_u8x4.h \
    common/image_u8x3.h \
    common/image_u8.h \
    common/image_types.h \
    common/image_f32.h \
    common/homography.h \
    common/getopt.h \
    common/g2d.h \
    common/floats.h \
    common/doubles_floats_impl.h \
    common/doubles.h

SOURCES += \
    tag36h11.c \
    tag36h10.c \
    tag36artoolkit.c \
    tag25h9.c \
    tag25h7.c \
    tag16h5.c \
    apriltag_quad_thresh.c \
    apriltag.c \
    common/zmaxheap.c \
    common/zhash.c \
    common/zarray.c \
    common/workerpool.c \
    common/unionfind.c \
    common/time_util.c \
    common/svd22.c \
    common/string_util.c \
    common/pnm.c \
    common/pjpeg-idct.c \
    common/pjpeg.c \
    common/pam.c \
    common/matd.c \
    common/image_u8x4.c \
    common/image_u8x3.c \
    common/image_u8.c \
    common/image_f32.c \
    common/homography.c \
    common/getopt.c \
    common/g2d.c \
    main.cpp
