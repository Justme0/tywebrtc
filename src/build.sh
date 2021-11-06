#!/usr/bin/env bash

set -ex

kServerName="server_tywebrtc" # svr name is same in Makefile

# format code
find | egrep "*\.(cpp|h)$" | xargs clang-format -i || true # we don't use *.cc or *.hpp and other extension. Format code forcely :)

# compile
make V=1

# deploy
rsync --port=22222 -vzrtp --progress --password-file=/data/home/taylorjiang/rsync/rsync.pass $kServerName devsync@9.218.129.75::workspace || true

# Could move to a single file meaning run server
runSvr()
{
    killall $kServerName || true
    cp /data/$kServerName .
    chmod +x $kServerName
    ./$kServerName &

    sleep 1

    echo ====================
    ps -eTo tid,pid,ppid,lstart,cmd | grep -v grep | grep $kServerName
    echo
    top -n 1 -b | grep $kServerName
    echo
    ss -anp | grep $kServerName
    echo ====================

    sleep 2
    tailf log.txt # is same as in code
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
