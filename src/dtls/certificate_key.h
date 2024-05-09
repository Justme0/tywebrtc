#ifndef DTLS_CERTIFICATE_KEY_H_
#define DTLS_CERTIFICATE_KEY_H_

#include "openssl/evp.h"
#include "openssl/x509.h"

namespace tywebrtc {

int GetCertificateAndKey(X509*& outCert, EVP_PKEY*& outKey);

}  // namespace tywebrtc

#endif  // DTLS_CERTIFICATE_KEY_H_
