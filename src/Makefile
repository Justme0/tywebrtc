# simple but bad rule :)

# use clang for showing compile error friendly
# svr name is same in build.sh
# NOTE: Mac OS must specify openssl directory, execute `brew info openssl` to show compile path
all:
	clang++ *cpp -std=c++11 -l ssl -l crypto -o server_tywebrtc -L /usr/local/opt/openssl@1.1/lib -I /usr/local/opt/openssl@1.1/include
