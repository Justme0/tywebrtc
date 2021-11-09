#ifndef CERTIFICATE_KEY_H
#define CERTIFICATE_KEY_H

#include <openssl/bio.h>
#include <openssl/pem.h>
#include <openssl/x509.h>

int GetCertificateAndKey(X509*& outCert, EVP_PKEY*& outKey);

#endif
