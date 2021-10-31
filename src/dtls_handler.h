#pragma once

class PeerConnection;

class DTLSHandler
{
public:
    DTLSHandler(PeerConnection& pc);
    int startDTLS();

private:
    PeerConnection& belongingPeerConnection_;
};
