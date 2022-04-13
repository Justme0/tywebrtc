#!/usr/bin/env bash

set -ex
ifconfig
date
whoami
w

g_pwd=`pwd`
trap 'cd $g_pwd' EXIT

g_dst_dir=$(cd $(dirname ${BASH_SOURCE[0]}); pwd )
# like golang defer
cd $g_dst_dir

# 1, run nodejs for web client demo
cd webclient

# only for my laptop :)
g_localip=`ifconfig  | grep wifi0 -A 1 | tail -n 1 | awk '{print $2}'`
sed "s/CANDIDATE_PLACEHOLDER/${g_localip}/g" index.html.template > index.html
# first kill, ignore error if not exist
ps aux | grep "node index.js" | grep -v grep | awk '{print $2}' | xargs kill -9 || true
# https://stackoverflow.com/questions/10408816/how-do-i-use-the-nohup-command-without-getting-nohup-out
node index.js &
cd $g_dst_dir

# 2, compile server
cd src
kServerName="server_tywebrtc" # svr name is same in Makefile

# force format code :)
# not emit failure if no clang-format
# mac (FreeBSD style) find cmd must specify directory
find . | egrep ".+\.(c|cc|h)$" | xargs clang-format -i --style Google || true

# compile
make V=1

# deploy
# rsync --port=22222 -vzrtp --progress --password-file=/data/home/taylorjiang/rsync/rsync.pass $kServerName devsync@9.218.129.75::workspace || true
cd $g_dst_dir

# 3, run server
cd src
# Could move to a single file meaning run server
runSvr()
{
    killall $kServerName || true # not emit error if no process running
    # cp /data/$kServerName .
    chmod +x $kServerName
    ./$kServerName &

    echo ====================
    # ps -eTo tid,pid,ppid,lstart,cmd | grep -v grep | grep $kServerName # ps option not support in mac os
    ps aux | grep -v grep | grep $kServerName
    echo
    top -n 1 -b | grep $kServerName || true # mac os ps have no -b option
    echo
    ss -anp | grep $kServerName || true # WSL executes `ss` may fail
    echo ====================

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
