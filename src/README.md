test: generate self certificate and private key
ref: https://stackoverflow.com/questions/10175812/how-to-generate-a-self-signed-ssl-certificate-using-openssl

```
$ openssl req -x509 -newkey rsa:4096 -keyout key.pem -out cert.pem -sha256 -days 365
```

data structure:
session(client IP port && ufrag) => peerConnection => multiple SSRC(or track)