# simple but bad rule :)

# use clang for showing compile error friendly
# svr name is same in build.sh
# NOTE: Mac OS must specify openssl directory, execute `brew info openssl` to show compile path
# TODO taylor if has clang++, use it. Otherwise use g++
# find and grep, NOTE escape $ in makefile

# https://github.com/cisco/libsrtp
# install libsrtp version 2
# e.g.
# Debian series: sudo apt install libsrtp2
# Mac: brew install srtp # not the name srtp

# -Wfatal-errors
CXXFLAGS += -g -Werror -Wall -Wextra -Wno-error=unused-parameter -Wno-error=unused-variable -Wno-error=missing-field-initializers -Wno-error=variadic-macros -Weffc++ -Wno-error=effc++ -D __STDC_FORMAT_MACROS

all:
	find . | egrep ".+\.cc$$" | xargs clang++ -std=c++17 ${CXXFLAGS} -I . -I ../third_party/tylib -l ssl -l crypto -l srtp2 -o server_tywebrtc -L /usr/local/opt/openssl@1.1/lib -I /usr/local/opt/openssl@1.1/include
