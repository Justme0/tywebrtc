#include "peer_connection.h"

#include "log/log.h"

PeerConnection::PeerConnection() : iceHandler_(*this), dtlsHandler_(*this), rtpHandler_(*this)
{
}

int PeerConnection::HandlePacket(const std::vector<char> &vBufReceive)
{
    int ret = 0;

    char cSubCmd = vBufReceive.front();
    PacketType packType = getPacketType(cSubCmd);
    tylog("subcmd=%d, packType=%d", cSubCmd, packType);
    tylog("stateMachine=%d", stateMachine_);

    switch (packType) {
    case PacketType::STUN: {
        // iUserNameLen = GetUfragFromIcePacket (pPackage, iLen, pUserName, REM_USER_NAME_LEN + 1);
        ret = iceHandler_.HandleIcePacket(vBufReceive);
        break;
    }

    default:
        break;
    }

    return ret;
}
