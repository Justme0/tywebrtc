#include "certificate_key.h"

#include <cstdio>

#include "openssl/bio.h"
#include "openssl/err.h"
#include "openssl/pem.h"
#include "src/log/log.h"

// OPT: 单例模式，服务器不用每次建PC都读文件
int GetCertificateAndKey(X509*& outCert, EVP_PKEY*& pkey) {
  int ret = 0;
  FILE* fp;
  static char certfile[] = "dtls/webrtc.cert";  // change path to ENV var
  static char keyfile[] = "dtls/webrtc.key";

  /* Read private key */

  fp = fopen(keyfile, "r");
  if (fp == nullptr) {
    tylog("shit no keyfile");
    exit(1);
  }
  pkey = PEM_read_PrivateKey(fp, nullptr, nullptr, nullptr);
  fclose(fp);

  if (pkey == nullptr) {
    ERR_print_errors_fp(stderr);
    tylog("shit");
    exit(1);
  }

  fp = fopen(certfile, "r");
  if (fp == nullptr) {
    tylog("shit no certfile");
    exit(1);
  }
  outCert = PEM_read_X509(fp, nullptr, nullptr, nullptr);
  fclose(fp);

  if (outCert == nullptr) {
    ERR_print_errors_fp(stderr);
    tylog("outCert err addr %p, cannot print msg?", stderr);
    tylog("shit");
    exit(1);
  }

  return ret;
}
