#!/usr/bin/env bash

set -ex

# format code
find | egrep "*\.(cpp|h)$" | xargs clang-format -i || true # we don't use *.cc or *.hpp and other extension. Format code forcely :)

# compile
make V=1

# deploy
rsync --port=22222 -vzrtp --progress --password-file=/data/home/taylorjiang/rsync/rsync.pass svr_tywebrtc devsync@9.218.129.75::workspace || true
