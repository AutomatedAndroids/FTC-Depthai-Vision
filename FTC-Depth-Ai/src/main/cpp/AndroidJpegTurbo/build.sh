# Built using NDK 21.3.6528147

#NDK_PATH=
TOOLCHAIN="clang"
ANDROID_VERSION="23"

# Clean up stuff
rm -rf bin > /dev/null
mkdir bin
mkdir bin/arm64-v8a
mkdir bin/armeabi-v7a
rm -rf build > /dev/null

# Build for ARM v7 with NEON
mkdir build
cd build
cmake -G"Unix Makefiles" \
  -DANDROID_ABI="armeabi-v7a with NEON" \
  -DANDROID_ARM_MODE=arm \
  -DANDROID_PLATFORM=android-${ANDROID_VERSION} \
  -DANDROID_TOOLCHAIN=${TOOLCHAIN} \
  -DCMAKE_ASM_FLAGS="--target=arm-linux-androideabi${ANDROID_VERSION}" \
  -DCMAKE_TOOLCHAIN_FILE=${NDK_PATH}/build/cmake/android.toolchain.cmake \
  ../libjpeg-turbo
make -j6

# Copy built file to bin and clean up
cp libturbojpeg.so ../bin/armeabi-v7a
cd ..
rm -rf build

# Build for ARM64-v8a
mkdir build
cd build
cmake -G"Unix Makefiles" \
  -DANDROID_ABI="arm64-v8a" \
  -DANDROID_ARM_MODE=arm \
  -DANDROID_PLATFORM=android-${ANDROID_VERSION} \
  -DANDROID_TOOLCHAIN=${TOOLCHAIN} \
  -DCMAKE_ASM_FLAGS="--target=arm-linux-androideabi${ANDROID_VERSION}" \
  -DCMAKE_TOOLCHAIN_FILE=${NDK_PATH}/build/cmake/android.toolchain.cmake \
  ../libjpeg-turbo
make -j6

# Copy built file to bin and clean up
cp libturbojpeg.so ../bin/arm64-v8a
cd ..
rm -rf build

# Strip built libs
${NDK_PATH}/toolchains/aarch64-linux-android-4.9/prebuilt/linux-x86_64/bin/aarch64-linux-android-strip bin/arm64-v8a/libturbojpeg.so
${NDK_PATH}/toolchains/arm-linux-androideabi-4.9/prebuilt/linux-x86_64/bin/arm-linux-androideabi-strip bin/armeabi-v7a/libturbojpeg.so
