#include "global_tmp/global_tmp.h"

#include "tylib/time/timer.h"

#include "log/log.h"
#include "pc/peer_connection.h"

// if construct map's value is expensive
// https://stackoverflow.com/questions/97050/stdmap-insert-or-stdmap-find
// here we can also use insert and update, but lower_bound is more general
// OPT: add ufrag logic
std::shared_ptr<PeerConnection> Singleton::GetPeerConnection(
    const std::string& ip, int port, const std::string& ufrag) {
  ClientSrcId clientSrcId{ip, port};
  auto lb = client2PC_.lower_bound(clientSrcId);
  tylog("client to pc map size=%zu", client2PC_.size());

  if (lb != client2PC_.end() &&
      !(client2PC_.key_comp()(clientSrcId, lb->first))) {
    assert(nullptr != lb->second);
    assert(lb->second->clientIP_ == ip && lb->second->clientPort_ == port);

    tylog("get old pc done");
    return lb->second;
  } else {
    // Use lb as a hint to insert, so it can avoid another lookup
    // OPT: ICE未选上的地址也会为它生成PC，可优化为PC池
    auto i = client2PC_.emplace_hint(
        lb, std::make_pair(clientSrcId, std::make_shared<PeerConnection>()));
    assert(nullptr != i->second);
    i->second->StoreClientIPPort(ip, port);

    tylog("new pc, ip=%s, port=%d, ufrag=%s", ip.data(), port, ufrag.data());
    return i->second;
  }
}

void Singleton::CleanTimeoutPeerConnection() {
  const int kPCDeadTimeoutMs = 20 * 1000;
  for (auto it = this->client2PC_.begin(); it != client2PC_.end();) {
    if (it->second->lastActiveTimeMs_ + kPCDeadTimeoutMs <
        static_cast<int64_t>(g_now_ms)) {
      tylog("timeout pc, clean it=%s.", it->second->ToString().data());
      it = client2PC_.erase(it);
    } else {
      ++it;
    }
  }
}