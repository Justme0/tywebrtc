#!/usr/bin/env bash

set -ex

kServerName="server_tywebrtc" # svr name is same in Makefile

# format code
# For C++ code we don't use *.cc, *.hpp or other extension. Force format code :)
# not emit failure if no clang-format
# mac (FreeBSD style) find cmd must specify directory
find . | egrep ".+\.(c|cpp|h)$" | xargs clang-format -i --style Google || true

# compile
make V=1

# deploy
# rsync --port=22222 -vzrtp --progress --password-file=/data/home/taylorjiang/rsync/rsync.pass $kServerName devsync@9.218.129.75::workspace || true

# Could move to a single file meaning run server
runSvr()
{
    killall $kServerName || true # not emit error if no process running
    # cp /data/$kServerName .
    chmod +x $kServerName
    ./$kServerName &

    echo ====================
    ps -eTo tid,pid,ppid,lstart,cmd | grep -v grep | grep $kServerName
    echo
    top -n 1 -b | grep $kServerName
    echo
    ss -anp | grep $kServerName || true # WSL execute ss fail
    echo ====================

    sleep 1
    tail -f log.txt # log file name is same as in code; some system have no tailf
}

# process cmd line argument
# https://stackoverflow.com/questions/192249/how-do-i-parse-command-line-arguments-in-bash
g_unknownOption=()
while [[ $# -gt 0 ]]; do
    key="$1"

    case $key in
        -r|--run)
            g_kIsRun=1
            shift # past argument
            ;;
        *)    # unknown option
            g_unknownOption+=("$1") # save it in an array for later
            shift # past argument, IMPORTANT!
            ;;
    esac
done

if [ -n "$g_kIsRun" ]; then
    runSvr
fi
