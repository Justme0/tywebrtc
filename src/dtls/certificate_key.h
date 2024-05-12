// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#ifndef SRC_DTLS_CERTIFICATE_KEY_H_
#define SRC_DTLS_CERTIFICATE_KEY_H_

#include "openssl/evp.h"
#include "openssl/x509.h"

namespace tywebrtc {

int GetCertificateAndKey(X509*& outCert, EVP_PKEY*& outKey);

}  // namespace tywebrtc

#endif  // SRC_DTLS_CERTIFICATE_KEY_H_
