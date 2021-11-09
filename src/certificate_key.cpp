#include "certificate_key.h"

#include "openssl/err.h"

#include <cstdio>

#include "log/log.h"

int GetCertificateAndKey(X509*& outCert, EVP_PKEY*& pkey) {
  int ret = 0;
  FILE* fp;
  static char certfile[] = "cert.pem";
  static char keyfile[] = "key.pem";

  /* Read private key */

  fp = fopen(keyfile, "r");
  if (fp == NULL) {
    tylog("shit");
    exit(1);
  }
  pkey = PEM_read_PrivateKey(fp, NULL, NULL, NULL);
  fclose(fp);

  if (pkey == NULL) {
    ERR_print_errors_fp(stderr);
    tylog("shit");
    exit(1);
  }

  fp = fopen(certfile, "r");
  if (fp == NULL) {
    tylog("shit");
    exit(1);
  }
  outCert = PEM_read_X509(fp, NULL, NULL, NULL);
  fclose(fp);

  if (outCert == NULL) {
    ERR_print_errors_fp(stderr);
    tylog("outCert err %s", stderr);
    tylog("shit");
    exit(1);
  }

  return ret;
}
