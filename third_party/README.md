# librtmp
we want use new openssl lib, but librtmp not support, e.g. librtmp define `HMAC_CTX` variable, but in new openssl lib, only provide pointer type (hide ABI), so we don't use openssl in rtmp :)
High GCC compiler has no `INT_MAX`? we redefine it to `INT32_MAX`
```
make XDEF="-DNO_SSL -DINT_MAX=INT32_MAX" CRYPTO=
```

# srt
```
cmake . -DENABLE_ENCRYPTION=OFF
make
```

make local server to recv SRT:
ffmpeg -f mpegts -i srt://127.0.0.1:9000?mode=listener -c copy out.ts -y -loglevel trace

# ffmpeg
https://trac.ffmpeg.org/wiki/CompilationGuide/Centos

```
sudo yum search opus
sudo yum install opus-devel
sudo yum install libvpx-devel
PATH="$HOME/bin:$PATH" PKG_CONFIG_PATH="$HOME/ffmpeg_build/lib/pkgconfig"  ./configure --enable-encoder=opus --enable-encoder=libopus --enable-libopus --enable-libvpx --enable-gpl --enable-libx264 --pkg-config-flags="--static" --extra-cflags="-I$HOME/ffmpeg_build/include" --extra-ldflags="-L$HOME/ffmpeg_build/lib"
make V=1
```
