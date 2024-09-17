// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.

#include "src/pc/peer_connection_manager.h"

namespace tywebrtc {

// OPT: merge with FindPeerPC
PeerConnection *PCManager::FindOnePeerPC() const {
  PeerConnection *pc = nullptr;

  for (auto &p : Singleton<PCManager>::Instance().pc_map_) {
    if (p.second.stateMachine_ < EnumStateMachine::GOT_RTP) {
      continue;
    }

    if (nullptr == pc || pc->lastActiveTimeMs_ < p.second.lastActiveTimeMs_) {
      pc = &p.second;
    }
  }

  if (nullptr != pc) {
    tylog("found peer=%s.", pc->ToString().data());
  } else {
    tylog("not found peer");
  }

  return pc;
}

// if construct map's value is expensive
// https://stackoverflow.com/questions/97050/stdmap-insert-or-stdmap-find
// here we can also use insert and update, but lower_bound is more general
// OPT: add ufrag logic
PeerConnection *PCManager::GetPeerConnection(
    const std::string &ip, int port, const std::vector<char> &vBufReceive) {
  int ret = 0;

  tylog("client to pc map size=%zu", pc_map_.size());

  ClientSrcId clientSrcId{ip, port};
  auto itUsername = addr2username_.find(clientSrcId);
  if (likely(itUsername != addr2username_.end())) {
    itUsername->second.lastActiveTimeMs = g_now_ms;
    tylog("get old pc done");
    // must exist
    return &pc_map_.at(itUsername->second.username);
  }

  // new client addr
  uint8_t cSubCmd = vBufReceive.front();
  PacketType packType = getPacketType(cSubCmd);
  if (PacketType::STUN != packType) {
    tylog("new addr first recv packtype=%s, should be stun.",
          PacketTypeToString(packType).data());

    return nullptr;
  }

  std::string username;
  bool bUseCandidate = false;
  ret =
      IceHandler::GetUfragFromIcePacket(vBufReceive, &username, &bUseCandidate);
  if (ret) {
    tylog("get ufrag ret=%d.", ret);
    assert(!"should not use assert :)");

    return nullptr;
  }

  tylog("recv stun of new client addr, username=%s, useCandidate=%d.",
        username.data(), bUseCandidate);
  assert(!username.empty() && "now only support handle stun bind req");

  addr2username_[clientSrcId] = PCKey(username, g_now_ms);

  auto itPC = pc_map_.find(username);
  if (itPC != pc_map_.end()) {
    if (bUseCandidate) {
      tylog("ICE switch candidate, client addr %s:%d -> %s:%d.",
            itPC->second.clientIP().data(), itPC->second.clientPort(),
            ip.data(), port);
      itPC->second.SetClientIPPort(ip, port);
    }

    return &itPC->second;
  }

  // could use `insert_or_assign`?
  auto itNewPC =
      pc_map_.emplace(std::piecewise_construct, std::forward_as_tuple(username),
                      std::forward_as_tuple(ip, port));
  tylog("new pc, ip=%s, port=%d.", ip.data(), port);
  assert(itNewPC.second);

  return &itNewPC.first->second;
}

// now no use
// get rtmp play fd, O(n).
/*
std::shared_ptr<PeerConnection> PCManager::GetPeerConnection(
    int targetFd) const {
  for (const std::pair<ClientSrcId, std::shared_ptr<PeerConnection>> &p :
       pc_map_) {
    if (p.second->pullHandler_.p_playSocket_ != nullptr &&
        *p.second->pullHandler_.p_playSocket_ == targetFd) {
      return p.second;
    }
  }

  return nullptr;
}
*/

void PCManager::CleanTimeoutPeerConnection() {
  for (auto it = this->pc_map_.begin(); it != pc_map_.end();) {
    if (it->second.lastActiveTimeMs_ + kPCDeadTimeoutMs <
        static_cast<int64_t>(g_now_ms)) {
      tylog("timeout pc, clean it=%s. after clean, pc_map size=%zu.",
            tylib::AnyToString(*it).data(), pc_map_.size() - 1);
      it = pc_map_.erase(it);
      // FIXME: destroy coroutine of the pc?
    } else {
      ++it;
    }
  }

  for (auto it = this->addr2username_.begin(); it != addr2username_.end();) {
    if (it->second.lastActiveTimeMs + kPCDeadTimeoutMs <
        static_cast<int64_t>(g_now_ms)) {
      tylog(
          "timeout addr2username slot, clean it=%s. after clean, addr2username "
          "map size=%zu.",
          it->second.ToString().data(), addr2username_.size() - 1);
      it = addr2username_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace tywebrtc
