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
# no -Wdeprecated-declarations is temp
CXXFLAGS += -g -Werror -Wall -Wextra -Wno-error=deprecated-declarations -Wno-error=unused-parameter -Wno-error=unused-variable -Wno-error=missing-field-initializers -Wno-error=variadic-macros -Weffc++ -Wno-error=effc++ -D __STDC_FORMAT_MACROS -std=c++17

# all:
# find . | egrep ".+\.cc$$" | xargs clang++ -std=c++17 ${CXXFLAGS} -I . -I ../third_party/tylib -l ssl -l crypto -l srtp2 -o server_tywebrtc -L /usr/local/opt/openssl@1.1/lib -I /usr/local/opt/openssl@1.1/include -I /opt/homebrew/opt/openssl@3/include -L /opt/homebrew/opt/openssl@3/lib -L /opt/homebrew/opt/srtp/lib



# http://make.mad-scientist.net/papers/advanced-auto-dependency-generation/#combine

SRCS := $(shell find . | egrep ".+\.cc$$")
$(info SRCS=$(SRCS))

OBJS := $(SRCS:%.cc=%.o)
$(info OBJS=$(OBJS))

DEPDIR := deps
DEPFLAGS = -MT $@ -MMD -MP -MF $(DEPDIR)/$*.d
CC = clang++

COMPILE.cc = $(CC) $(DEPFLAGS) $(CFLAGS) $(CXXFLAGS) $(TARGET_ARCH) -c

all: $(OBJS)
	$(CC) $(CXXFLAGS) $< -l ssl -l crypto -l srtp2 -L /usr/local/opt/openssl@1.1/lib -L /opt/homebrew/opt/openssl@3/lib -L /opt/homebrew/opt/srtp/lib -o server_tywebrtc

%.o : %.cc
%.o : %.cc $(DEPDIR)/%.d | $(DEPDIR)
		$(COMPILE.cc) $(OUTPUT_OPTION) $<

$(DEPDIR): ; @mkdir -p $@

DEPFILES := $(SRCS:%.cc=$(DEPDIR)/%.d)
$(info DEPFILES=$(DEPFILES))
$(DEPFILES):
include $(wildcard $(DEPFILES))