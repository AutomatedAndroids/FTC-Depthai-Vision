# Sets the minimum version of CMake required to build your native library.
# This ensures that a certain set of CMake features is available to
# your build.

cmake_minimum_required(VERSION 3.4.1)
# cmake_policy(SET CMP01111 NEW)
project("FTC Depth AI")
set (CMAKE_CXX_STANDARD 11)

SET(CMAKE_SYSTEM_NAME Android)
SET(CMAKE_SYSTEM_VERSION 24)
SET(CMAKE_ANDROID_ARCH_ABI armeabi-v7a)

# Specifies a library name, specifies whether the library is STATIC or
# SHARED, and provides relative paths to the source code. You can
# define multiple libraries by adding multiple add_library() commands,
# and CMake builds them for you. When you build your app, Gradle
# automatically packages shared libraries with your APK.

add_library( # Specifies the name of the library.
        EasyOpenCV

        # Sets the library as a shared library.
        SHARED

        # Provides a relative path to your source file(s).
        openftc/easyopencv/camera2_jni.cpp
        openftc/easyopencv/camera2_jni.cpp
        openftc/easyopencv/camera2_jni.cpp)
add_library(# Specifies the name of the library.
        apriltag

        # Sets the library as a shared library.
        SHARED

        # Provides a relative path to your source file(s).
        openftc/apriltag/src/main/cpp/AprilTagDetectorJNI.cpp
        openftc/apriltag/src/main/cpp/AprilTagDetectionJNI.cpp

        openftc/apriltag/apriltag/common/g2d.c
        openftc/apriltag/apriltag/common/getopt.c
        openftc/apriltag/apriltag/common/homography.c
        openftc/apriltag/apriltag/common/image_u8.c
        openftc/apriltag/apriltag/common/image_u8x3.c
        openftc/apriltag/apriltag/common/image_u8x4.c
        openftc/apriltag/apriltag/common/matd.c
        openftc/apriltag/apriltag/common/pam.c
        openftc/apriltag/apriltag/common/pjpeg.c
        openftc/apriltag/apriltag/common/pjpeg-idct.c
        openftc/apriltag/apriltag/common/pnm.c
        openftc/apriltag/apriltag/common/string_util.c
        openftc/apriltag/apriltag/common/svd22.c
        openftc/apriltag/apriltag/common/time_util.c
        openftc/apriltag/apriltag/common/unionfind.c
        openftc/apriltag/apriltag/common/workerpool.c
        openftc/apriltag/apriltag/common/zarray.c
        openftc/apriltag/apriltag/common/zhash.c
        openftc/apriltag/apriltag/common/zmaxheap.c
        openftc/apriltag/apriltag/apriltag.c
        openftc/apriltag/apriltag/apriltag_pose.c
        openftc/apriltag/apriltag/apriltag_quad_thresh.c
        openftc/apriltag/apriltag/tag16h5.c
        openftc/apriltag/apriltag/tag25h9.c
        openftc/apriltag/apriltag/tag36h11.c
        openftc/apriltag/apriltag/tagCircle21h7.c
        openftc/apriltag/apriltag/tagStandard41h12.c
)

# Specifies a path to native header files.
include_directories(src/main/cpp/include/
        ${CMAKE_CURRENT_SOURCE_DIR}/OpenCV-Repackaged/doc/native_headers/
        AndroidJpegTurbo/libjpeg-turbo/)

# Specify path to prebuilt object
set(OPENCV_NATIVE_LIB_PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/OpenCV-Repackaged/OpenCV-Android-SDK/src/main/jniLibs)

add_library(openCvNative SHARED IMPORTED)
set_target_properties(openCvNative PROPERTIES IMPORTED_LOCATION
        ${OPENCV_NATIVE_LIB_PARENT_DIR}/${ANDROID_ABI}/libopencv_java4.so)

add_library(turbojpeg SHARED IMPORTED)
set_target_properties(turbojpeg PROPERTIES IMPORTED_LOCATION
        C:/Users/caden/StudioProjects/FTC-Depthai-Vision/FTC-Depth-Ai/src/main/cpp/AndroidJpegTurbo/bin/${ANDROID_ABI}/libturbojpeg.so
        #        ${CMAKE_CURRENT_SOURCE_DIR}/AndroidJpegTurbo/bin/${ANDROID_ABI}/libturbojpeg.so
)

# linking libs
target_link_libraries(
        EasyOpenCV
        apriltag
        turbojpeg
        openCvNative
        log
        android)

# FINDS JNI AND APPLYS IT

IF (DEFINED $ENV{JAVA_HOME})
    SET(JAVA_HOME,$ENV{JAVA_HOME})
    SET(JNI_LOC,$ENV{JAVA_HOME}+"/include")
ELSE ()
    FIND_PACKAGE(Java)
    if (JNI_FOUND)
        set(JNI_LOC,${JNI_INCLUDE_DIRS})
    ENDIF ()
ENDIF ()
if (DEFINED JNI_LOC)
    include_directories("C:/Users/caden/.jdks/openjdk-21.0.1/include")
    if ($ENV{OS} EQUAL "WINDOWS-NT" OR ${CMAKE_HOST_WIN32})
        include_directories("C:/Users/caden/.jdks/openjdk-21.0.1/include/win32")
    ENDIF ()
endif ()
include_directories(C:/Users/caden/AppData/Local/Android/Sdk/ndk/21.3.6528147/toolchains/llvm/prebuilt/windows-x86_64/sysroot/usr/include)
# include_directories("C:/Users/caden/.jdks/openjdk-21.0.1/include")
# include_directories("C:/Users/caden/.jdks/openjdk-21.0.1/include/win32")
