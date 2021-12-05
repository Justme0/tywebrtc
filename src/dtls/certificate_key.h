#ifndef DTLS_CERTIFICATE_KEY_H_
#define DTLS_CERTIFICATE_KEY_H_

#include "openssl/evp.h"
#include "openssl/x509.h"

int GetCertificateAndKey(X509*& outCert, EVP_PKEY*& outKey);

#endif  // DTLS_CERTIFICATE_KEY_H_
