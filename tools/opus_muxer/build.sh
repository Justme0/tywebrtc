set -ex

# sudo yum update
# sudo yum install bzip2 gnutls -y

clang-format -i opus_muxer.cpp
cd $(dirname "$0")

# scl -l
# yum list all --enablerepo='centos-sclo-rh'
scl enable devtoolset-8 -- g++ -std=c++17 \
    -I ../../third_party/tylib \
    -I ../../third_party/ffmpeg \
    opus_muxer.cpp \
    ../../third_party/ffmpeg/libavformat/libavformat.a \
    ../../third_party/ffmpeg/libavcodec/libavcodec.a \
    ../../third_party/ffmpeg/libswresample/libswresample.a \
    ../../third_party/ffmpeg/libswscale/libswscale.a \
    ../../third_party/ffmpeg/libavutil/libavutil.a \
    ../../third_party/opus/.libs/libopus.a \
    ../../third_party/x264/libx264.a \
    ../../third_party/libvpx/libvpx.a \
    -l dl -pthread -l z -l bz2 -l gnutls \
    -o opus_muxer
