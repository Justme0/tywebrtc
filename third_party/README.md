# ffmpeg
https://trac.ffmpeg.org/wiki/CompilationGuide/Centos

sudo yum search opus
sudo yum install opus-devel
sudo yum install libvpx-devel
PATH="$HOME/bin:$PATH" PKG_CONFIG_PATH="$HOME/ffmpeg_build/lib/pkgconfig"  ./configure --enable-encoder=opus --enable-encoder=libopus --enable-libopus --enable-libvpx --enable-gpl --enable-libx264 --pkg-config-flags="--static" --extra-cflags="-I$HOME/ffmpeg_build/include" --extra-ldflags="-L$HOME/ffmpeg_build/lib"
make V=1
