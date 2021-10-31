#pragma once

#include "ice_handler.h"
#include "dtls_handler.h"
#include "rtp_handler.h"

enum class PacketType
{
    STUN,
    DTLS,
    RTP,
    UNKNOWN,
};

inline PacketType getPacketType (unsigned char cSubCmd)
{
    if ((cSubCmd == 0)   || (cSubCmd == 1))
        return PacketType::STUN;

    if ((cSubCmd >= 20)  && (cSubCmd <= 64))
        return PacketType::DTLS;

    if ((cSubCmd >= 128) && (cSubCmd <= 191))
        return PacketType::RTP;

    return PacketType::UNKNOWN;
}

enum class EnumStateMachine {
    SDP_DONE,
    GET_CANDIDATE_DONE,
    ICE_USE_CANDIDATE_DONE,
    //EnumStateMachine_ICE_START,
    ICE_DONE,
    //EnumStateMachine_DTLS_START,
    DTLS_DONE,
    //EnumStateMachine_RTP_START,
};

class PeerConnection
{
public:
    PeerConnection();
    enum EnumStateMachine stateMachine_;

    int HandlePacket(const std::vector<char> &vBufReceive);

//private:
    ICEHandler iceHandler_;
    DTLSHandler dtlsHandler_;
    RTPHandler rtpHandler_;

    std::string peerClientIP_;
    int peerClientPort_;
};
