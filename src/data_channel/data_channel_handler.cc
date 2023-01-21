#include "data_channel/data_channel_handler.h"

#include <fcntl.h>
#include <cstdarg>
#include <cstdlib>
#include <cstring>
#include <sstream>

#include "log/log.h"

enum DataChannelMessageType {
  DataChannelMessageTypeAck = 2,
  DataChannelMessageTypeOpen = 3,
};

enum DataChannelChannelType {
  DataChannelChannelTypeReliable = 0x00,
  DataChannelChannelTypeReliableUnordered = 0x80,
  DataChannelChannelTypePartialReliableRexmit = 0x01,
  DataChannelChannelTypePartialReliableRexmitUnordered = 0x81,
  DataChannelChannelTypePartialReliableTimed = 0x02,
  DataChannelChannelTypePartialReliableTimedUnordered = 0x82,
};

enum DataChannelPPID {
  DataChannelPPIDControl = 50,
  DataChannelPPIDString = 51,
  DataChannelPPIDBinary = 53,
};

uint16_t event_types[] = {SCTP_ADAPTATION_INDICATION, SCTP_ASSOC_CHANGE,
                          SCTP_ASSOC_RESET_EVENT,     SCTP_REMOTE_ERROR,
                          SCTP_SHUTDOWN_EVENT,        SCTP_SEND_FAILED_EVENT,
                          SCTP_STREAM_RESET_EVENT,    SCTP_STREAM_CHANGE_EVENT};

SctpGlobalEnv* g_sctp_env = nullptr;

const int kSctpSuccess = 1;
const int kSctpPort = 5000;
const int kMaxStream = 128;

static void SctpDebugLog(const char* format, ...) {
  char buffer[4096];
  va_list ap;

  va_start(ap, format);
  int nb = vsnprintf(buffer, sizeof(buffer), format, ap);
  if (nb > 0) {
    if (buffer[nb - 1] == '\n') {
      if (nb >= 2 && buffer[nb - 2] == '\r') {
        buffer[nb - 2] = '\0';
      } else {
        buffer[nb - 1] = '\0';
      }
    } else {
      buffer[nb] = '\0';
    }
    tylog("%s", buffer);
  }

  va_end(ap);
}

static DataChannelHandler* GetSctpFromSocket(struct socket* sock) {
  struct sockaddr* addrs = nullptr;
  int naddrs = usrsctp_getladdrs(sock, 0, &addrs);
  if (naddrs <= 0 || addrs[0].sa_family != AF_CONN) {
    return nullptr;
  }

  struct sockaddr_conn* sconn =
      reinterpret_cast<struct sockaddr_conn*>(&addrs[0]);
  DataChannelHandler* sctp =
      g_sctp_env->Get(reinterpret_cast<uintptr_t>(sconn->sconn_addr));
  usrsctp_freeladdrs(addrs);

  return sctp;
}

static int RecvSctpDataCallback(struct socket* sock,
                                union sctp_sockstore /*addr*/, void* data,
                                size_t len, struct sctp_rcvinfo rcv, int flags,
                                void* ulp_info) {
  DataChannelHandler* sctp = GetSctpFromSocket(sock);

  if (sctp == nullptr) {
    tylog("no found sctp");
    if (data) {
      free(data);
    }
    return -1;
  }

  if (sctp != static_cast<DataChannelHandler*>(ulp_info)) {
    tylog("sctp no match, maybe error");
    if (data) {
      free(data);
    }
    return -1;
  }

  if (flags & MSG_NOTIFICATION) {
    sctp->OnSctpEvent(rcv, data, len);
  } else {
    sctp->OnSctpData(rcv, data, len);
  }
  if (data) {
    free(data);
  }
  return kSctpSuccess;
}

static int SendSctpDataCallback(void* addr, void* data, size_t len,
                                uint8_t /*tos*/, uint8_t /*set_df*/) {
  DataChannelHandler* sctp = g_sctp_env->Get(reinterpret_cast<uintptr_t>(addr));

  if (sctp == nullptr) {
    tylog("no found sctp");
    return -1;
  }

  sctp->OnSendSctpData(reinterpret_cast<const uint8_t*>(data), len);

  return 0;
}

static int SendThresholdCallback(struct socket* sock, uint32_t sb_free,
                                 void* /*ulp_info*/) {
  DataChannelHandler* sctp = GetSctpFromSocket(sock);

  if (sctp == nullptr) {
    tylog("no found sctp");
    return -1;
  }

  tylog("sb_free=%u", sb_free);

  sctp->OnSendThresholdCallback();

  return 0;
}

DataChannel::DataChannel() {
  label_ = "";
  sid_ = 0;

  channel_type_ = 0;
  reliability_params_ = 0;
  status_ = DataChannelStatusClosed;
};

SctpGlobalEnv::SctpGlobalEnv() {
  id_ = 0;
  tylog("sctp global env init");
  usrsctp_init(0, SendSctpDataCallback, SctpDebugLog);

  usrsctp_sysctl_set_sctp_ecn_enable(0);
  usrsctp_sysctl_set_sctp_asconf_enable(0);
  usrsctp_sysctl_set_sctp_auth_enable(0);
  usrsctp_sysctl_set_sctp_nr_outgoing_streams_default(kMaxStream);

  // usrsctp_sysctl_set_sctp_debug_on(SCTP_DEBUG_ALL);
}

SctpGlobalEnv::~SctpGlobalEnv() { usrsctp_finish(); }

uintptr_t SctpGlobalEnv::Register(DataChannelHandler* sctp) {
  std::unique_lock<std::mutex> lock_guard(sctp_map_mutex_);

  auto id = ++id_;
  sctp_map_.insert(std::make_pair(id, sctp));

  return id;
}

DataChannelHandler* SctpGlobalEnv::Get(uintptr_t id) {
  std::unique_lock<std::mutex> lock_guard(sctp_map_mutex_);

  auto iter = sctp_map_.find(id);
  if (iter == sctp_map_.end()) {
    return nullptr;
  }

  return iter->second;
}

bool SctpGlobalEnv::UnRegister(uintptr_t id) {
  std::unique_lock<std::mutex> lock_guard(sctp_map_mutex_);
  return sctp_map_.erase(id) > 0;
}

DataChannelHandler::DataChannelHandler(PeerConnection& pc)
    : belongingPeerConnection_(pc) {
  if (g_sctp_env == nullptr) {
    g_sctp_env = new SctpGlobalEnv();
  }

  id_ = g_sctp_env->Register(this);

  data_channel_id_ = 1;

  send_blocking_ = false;

  tylog("taylor");

  usrsctp_register_address(reinterpret_cast<void*>(id_));
  sctp_socket_ =
      usrsctp_socket(AF_CONN, SOCK_STREAM, IPPROTO_SCTP, RecvSctpDataCallback,
                     SendThresholdCallback, 0, this);

  int ret = usrsctp_set_non_blocking(sctp_socket_, 1);
  if (ret < 0) {
    tylog("usrrsctp set non blocking failed, ret=%d, err=%d(%s)", ret, errno,
          strerror(errno));
  }

  struct sctp_assoc_value av;

  av.assoc_value = SCTP_ENABLE_RESET_STREAM_REQ | SCTP_ENABLE_RESET_ASSOC_REQ |
                   SCTP_ENABLE_CHANGE_ASSOC_REQ;
  ret = usrsctp_setsockopt(sctp_socket_, IPPROTO_SCTP, SCTP_ENABLE_STREAM_RESET,
                           &av, sizeof(av));
  if (ret < 0) {
    tylog("usrrsctp set SCTP_ENABLE_STREAM_RESET failed, ret=%d, err=%d(%s)",
          ret, errno, strerror(errno));
  }

  uint32_t no_delay = 1;
  ret = usrsctp_setsockopt(sctp_socket_, IPPROTO_SCTP, SCTP_NODELAY, &no_delay,
                           sizeof(no_delay));
  if (ret < 0) {
    tylog("usrrsctp set SCTP_NODELAY failed, ret=%d, err=%d(%s)", ret, errno,
          strerror(errno));
  }

  uint32_t eor = 1;
  if (usrsctp_setsockopt(sctp_socket_, IPPROTO_SCTP, SCTP_EXPLICIT_EOR, &eor,
                         sizeof(eor))) {
    tylog("usrrsctp set SCTP_EXPLICIT_EOR failed, ret=%d, err=%d(%s)", ret,
          errno, strerror(errno));
  }

  struct sctp_event event;
  memset(&event, 0, sizeof(event));
  event.se_on = 1;

  for (size_t i = 0; i < sizeof(event_types) / sizeof(uint16_t); ++i) {
    event.se_type = event_types[i];
    ret = usrsctp_setsockopt(sctp_socket_, IPPROTO_SCTP, SCTP_EVENT, &event,
                             sizeof(event));
    if (ret < 0) {
      tylog("usrrsctp set SCTP_NODELAY failed, ret=%d, err=%d(%s)", ret, errno,
            strerror(errno));
    }
  }

  tylog("taylor");
  // Init message.
  struct sctp_initmsg initmsg;
  memset(&initmsg, 0, sizeof(initmsg));
  initmsg.sinit_num_ostreams = kMaxStream;
  initmsg.sinit_max_instreams = kMaxStream;

  tylog("taylor");
  ret = usrsctp_setsockopt(sctp_socket_, IPPROTO_SCTP, SCTP_INITMSG, &initmsg,
                           sizeof(initmsg));
  if (ret < 0) {
    tylog("usrrsctp set SCTP_INITMSG failed, ret=%d, err=%d(%s)", ret, errno,
          strerror(errno));
  }

  struct sockaddr_conn sconn;
  memset(&sconn, 0, sizeof(sconn));
  sconn.sconn_family = AF_CONN;
  sconn.sconn_port = htons(kSctpPort);
  sconn.sconn_addr = reinterpret_cast<void*>(id_);

  ret = usrsctp_bind(sctp_socket_, reinterpret_cast<struct sockaddr*>(&sconn),
                     sizeof(sconn));
  if (ret < 0) {
    tylog("usrrsctp bind failed, ret=%d, err=%d(%s), this=%p", ret, errno,
          strerror(errno), this);
  }
  tylog("taylor");

  // taylor
  ConnectToClass();
}

DataChannelHandler::~DataChannelHandler() {
  std::unique_lock<std::mutex> lock_guard(sctp_mutex_);
  tylog("taylor");
  if (sctp_socket_) {
    usrsctp_close(sctp_socket_);
    usrsctp_deregister_address(reinterpret_cast<void*>(id_));
    g_sctp_env->UnRegister(id_);
  }
  tylog("taylor");
}

// only called by constructor, no need use lock
int DataChannelHandler::ConnectToClass() {
  struct sockaddr_conn rconn;
  memset(&rconn, 0, sizeof(rconn));
  rconn.sconn_family = AF_CONN;
  rconn.sconn_port = htons(kSctpPort);
  rconn.sconn_addr = reinterpret_cast<void*>(id_);

  int ret = usrsctp_connect(
      sctp_socket_, reinterpret_cast<struct sockaddr*>(&rconn), sizeof(rconn));
  if (ret < 0 && errno != EINPROGRESS) {
    tylog("usrrsctp connect failed, ret=%d, err=%d(%s)", ret, errno,
          strerror(errno));
    return -1;
  }

  struct sctp_paddrparams peer_addr_param;
  memset(&peer_addr_param, 0, sizeof(peer_addr_param));
  memcpy(&peer_addr_param.spp_address, &rconn, sizeof(rconn));
  peer_addr_param.spp_flags = SPP_PMTUD_DISABLE;
  peer_addr_param.spp_pathmtu = 1200 - sizeof(struct sctp_common_header);

  ret = usrsctp_setsockopt(sctp_socket_, IPPROTO_SCTP, SCTP_PEER_ADDR_PARAMS,
                           &peer_addr_param, sizeof(peer_addr_param));
  if (ret < 0) {
    tylog("usrrsctp set SCTP_PEER_ADDR_PARAMS failed, ret=%d, err=%d(%s)", ret,
          errno, strerror(errno));
    return -1;
  }

  tylog("usrsctp connect peer success.");

  return 0;
}

void DataChannelHandler::Feed(const uint8_t* buf, const int nb_buf) {
  usrsctp_conninput(reinterpret_cast<void*>(id_), buf, nb_buf, 0);
}

int DataChannelHandler::CreateDataChannel(const std::string& label) {
  DataChannel data_channel;
  data_channel.label_ = label;
  data_channel.sid_ = data_channel_id_++;
  data_channel.channel_type_ = DataChannelChannelTypeReliable;
  data_channel.reliability_params_ = 0;
  data_channel.status_ = DataChannelStatusOpen;

  label_sid_.insert(std::make_pair(data_channel.label_, data_channel.sid_));
  data_channels_.insert(std::make_pair(data_channel.sid_, data_channel));

  int reqlen = sizeof(struct DcepOpenMessage) + label.size();
  struct DcepOpenMessage* req = (struct DcepOpenMessage*)calloc(1, reqlen);
  struct sctp_sndinfo sndinfo;
  req->message_type = DataChannelMessageTypeOpen;
  req->channel_type = DataChannelChannelTypeReliable;
  req->priority = htons(0);                     /* XXX: add support */
  req->reliability_params = htons((uint16_t)0); /* XXX Why 16-bit */
  if (!label.empty()) {
    req->label_length = htons(label.size());
    memcpy(req->label_and_protocol, label.c_str(), label.size());
  }

  memset(&sndinfo, 0, sizeof(struct sctp_sndinfo));
  sndinfo.snd_sid = data_channel.sid_;
  sndinfo.snd_flags = SCTP_EOR;
  sndinfo.snd_ppid = htonl(DataChannelPPIDControl);

  if (usrsctp_sendv(sctp_socket_, req, reqlen, nullptr, 0, &sndinfo,
                    (socklen_t)sizeof(struct sctp_sndinfo), SCTP_SENDV_SNDINFO,
                    0) < 0) {
    tylog("sctp_sendv open request failed.");
    free(req);
    return -1;
  }

  free(req);

  return 0;
}

int DataChannelHandler::OnSctpEvent(const struct sctp_rcvinfo&, void* data,
                                    size_t len) {
  union sctp_notification* sctp_notify =
      reinterpret_cast<union sctp_notification*>(data);
  if (sctp_notify->sn_header.sn_length != len) {
    tylog("sctp notify header");
    return -1;
  }

  tylog("sctp event type=%d", (int)sctp_notify->sn_header.sn_type);

  switch (sctp_notify->sn_header.sn_type) {
    case SCTP_ASSOC_CHANGE:
      break;
    case SCTP_REMOTE_ERROR:
      break;
    case SCTP_SHUTDOWN_EVENT:
      break;
    case SCTP_ADAPTATION_INDICATION:
      break;
    case SCTP_PARTIAL_DELIVERY_EVENT:
      break;
    case SCTP_AUTHENTICATION_EVENT:
      break;
    case SCTP_SENDER_DRY_EVENT:
      break;
    case SCTP_NOTIFICATIONS_STOPPED_EVENT:
      break;
    case SCTP_SEND_FAILED_EVENT: {
      const struct sctp_send_failed_event& ssfe =
          sctp_notify->sn_send_failed_event;
      tylog("SCTP_SEND_FAILED_EVENT, ppid=%u, sid=%u, flags=0x%04x, err=0x%08x",
            ntohl(ssfe.ssfe_info.snd_ppid), ssfe.ssfe_info.snd_sid,
            ssfe.ssfe_info.snd_flags, ssfe.ssfe_error);
      break;
    }
    case SCTP_STREAM_RESET_EVENT:
      break;
    case SCTP_ASSOC_RESET_EVENT:
      break;
    case SCTP_STREAM_CHANGE_EVENT:
      break;
    case SCTP_PEER_ADDR_CHANGE:
      break;
    default:
      break;
  }

  return kSctpSuccess;
}

void DataChannelHandler::OnSendThresholdCallback() {
  if (send_blocking_) {
    if (SendBufferedMsg() != 0) {
      tylog("send still blocking");
      return;
    } else {
      tylog("buffered msg sended");
      send_blocking_ = false;
    }
  }
}

void DataChannelHandler::OnSendSctpData(const uint8_t* data, size_t len) {
  tylog("send sctp data=%p, len=%zu.", data, len);
  // OnSendToDtlsChannel
}

int DataChannelHandler::OnSctpData(const struct sctp_rcvinfo& rcv, void* data,
                                   size_t len) {
  int err = 0;

  uint32_t ppid = ntohl(rcv.rcv_ppid);
  switch (ppid) {
    case DataChannelPPIDControl:
      err = OnDataChannelControl(rcv, (char*)data, len);
      break;
    case DataChannelPPIDString:
    case DataChannelPPIDBinary:
      err = OnDataChannelMsg(rcv, (char*)data, len);
      break;
    default:
      break;
  }
  return err;
}

int DataChannelHandler::OnDataChannelControl(const struct sctp_rcvinfo& rcv,
                                             char* data, int len) {
  if (len <= 12) {
    tylog("sctp data length invalid");
    return -1;
  }

  int pos = 0;
  uint32_t byte_left = len;
  uint8_t msg_type = data[pos];
  pos += 1;
  switch (msg_type) {
    case DataChannelMessageTypeOpen: {
      if (data_channels_.count(rcv.rcv_sid)) {
        tylog("data channel already opened.");
        return 0;
      }

      uint8_t channel_type = data[pos];
      pos += 1;
      uint16_t priority = ((data[pos] << 8) | (data[pos + 1]));
      pos += 2;
      uint32_t reliability_params = ((data[pos] << 24) | (data[pos + 1] << 16) |
                                     (data[pos + 2] << 8) | data[pos + 3]);
      pos += 4;
      uint16_t label_length = ((data[pos] << 8) | (data[pos + 1]));
      pos += 2;
      uint16_t protocol_length = ((data[pos] << 8) | (data[pos + 1]));
      pos += 2;

      tylog(
          "channel_type=%u, priority=%u, "
          "reliability_params=%u, label_length=%u, "
          "protocol_length=%u",
          channel_type, priority, reliability_params, label_length,
          protocol_length);

      byte_left -= 12;

      switch (channel_type) {
        case DataChannelChannelTypeReliable: {
          tylog("reliable channel");
          break;
        }
        case DataChannelChannelTypeReliableUnordered: {
          tylog("unordered channel");
          break;
        }
        case DataChannelChannelTypePartialReliableRexmit: {
          tylog("ordered channel with max rexmit %u", reliability_params);
          break;
        }
        case DataChannelChannelTypePartialReliableRexmitUnordered: {
          tylog("unordered channel with max rexmit %u", reliability_params);
          break;
        }
        case DataChannelChannelTypePartialReliableTimed: {
          tylog("ordered channel with max life time %u", reliability_params);
          break;
        }
        case DataChannelChannelTypePartialReliableTimedUnordered: {
          tylog("unordered channel with max life time %u", reliability_params);
          break;
        }
      }

      std::string label = "";
      if (label_length > 0) {
        if (label_length > byte_left) {
          tylog("sctp data length invalid");
          return -1;
        }

        label.assign(data + pos, label_length);
        byte_left -= label_length;
        pos += label_length;
      }

      if (protocol_length > 0) {
        if (protocol_length > byte_left) {
          tylog("sctp data length invalid");
          return -1;
        }
      }

      tylog(
          "channel_type=%u, priority=%u, reliability_params=%u, "
          "label_length=%u, "
          "protocol_length=%u, label=%s",
          channel_type, priority, reliability_params, label_length,
          protocol_length, label.c_str());

      DataChannel data_channel;
      data_channel.label_ = label;
      data_channel.sid_ = rcv.rcv_sid;
      data_channel.channel_type_ = channel_type;
      data_channel.reliability_params_ = reliability_params;
      data_channel.status_ = DataChannelStatusOpen;

      label_sid_.insert(std::make_pair(data_channel.label_, data_channel.sid_));
      data_channels_.insert(std::make_pair(data_channel.sid_, data_channel));

      tylog("log datachannel open: label:%s, sid : %u", label.c_str(),
            rcv.rcv_sid);

      break;
    }
    case DataChannelMessageTypeAck: {
      break;
    }
    default: { tylog("unknown data channel control msg type"); }
  }

  return 0;
}

int DataChannelHandler::OnDataChannelMsg(const struct sctp_rcvinfo& rcv,
                                         char* data, int len) {
  std::string label = "";
  std::unique_lock<std::mutex> lock_guard(channel_mutex_);
  std::map<uint16_t, DataChannel>::iterator iter =
      data_channels_.find(rcv.rcv_sid);
  if (iter == data_channels_.end()) {
    tylog("can not found sid=%d", rcv.rcv_sid);
    return -1;
  }
  label = iter->second.label_;
  lock_guard.unlock();

  tylog("on recv data=%p, len=%d", data, len);

  // callback OnRecvDataChannelMsg

  return 0;
}

int DataChannelHandler::Send(const std::string& label, const uint8_t* buf,
                             const int len) {
  uint16_t sid = 0;
  std::unique_lock<std::mutex> lock_guard(channel_mutex_);
  std::map<std::string, uint16_t>::iterator iter = label_sid_.find(label);
  if (iter == label_sid_.end()) {
    tylog("can not found label=%s", label.c_str());
    return kChannelNotFound;
  }
  sid = iter->second;
  lock_guard.unlock();

  return Send(sid, buf, len);
}

sctp_sendv_spa DataChannelHandler::CreateSendParam(
    const DataChannel& data_channel) {
  sctp_sendv_spa spa;  // NOTE no init

  spa.sendv_flags = SCTP_SEND_SNDINFO_VALID;
  spa.sendv_sndinfo.snd_sid = data_channel.sid_;
  spa.sendv_sndinfo.snd_ppid = htonl(DataChannelPPIDString);
  spa.sendv_sndinfo.snd_flags = SCTP_EOR;

  if (data_channel.channel_type_ & 0x80) {
    spa.sendv_sndinfo.snd_flags |= SCTP_UNORDERED;
  }

  if (data_channel.channel_type_ ==
          DataChannelChannelTypePartialReliableRexmitUnordered ||
      data_channel.channel_type_ ==
          DataChannelChannelTypePartialReliableRexmit) {
    spa.sendv_prinfo.pr_policy = SCTP_PR_SCTP_RTX;
    spa.sendv_prinfo.pr_value = data_channel.reliability_params_;
    spa.sendv_flags |= SCTP_SEND_PRINFO_VALID;
  } else if (data_channel.channel_type_ ==
                 DataChannelChannelTypePartialReliableTimedUnordered ||
             data_channel.channel_type_ ==
                 DataChannelChannelTypePartialReliableTimed) {
    spa.sendv_prinfo.pr_policy = SCTP_PR_SCTP_TTL;
    spa.sendv_prinfo.pr_value = data_channel.reliability_params_;
    spa.sendv_flags |= SCTP_SEND_PRINFO_VALID;
  } else {
    spa.sendv_prinfo.pr_policy = SCTP_PR_SCTP_NONE;
    spa.sendv_prinfo.pr_value = 0;
  }

  return spa;
}

int DataChannelHandler::SendInternal(const uint16_t sid, const uint8_t* buf,
                                     const int len) {
  DataChannel data_channel;
  std::unique_lock<std::mutex> lock_guard(channel_mutex_);
  std::map<uint16_t, DataChannel>::iterator iter = data_channels_.find(sid);
  if (iter == data_channels_.end()) {
    tylog("can not found sid=%d", sid);
    return -1;
  }

  data_channel = iter->second;
  lock_guard.unlock();

  if (data_channel.status_ != DataChannelStatusOpen) {
    tylog("data channel %d no opened", sid);
    return -1;
  }

  sctp_sendv_spa spa = CreateSendParam(data_channel);

  std::unique_lock<std::mutex> sctp_lock_guard(sctp_mutex_);
  ssize_t send_res = usrsctp_sendv(sctp_socket_, buf, len, nullptr, 0, &spa,
                                   sizeof(spa), SCTP_SENDV_SPA, 0);

  tylog("stcp send %d, ret=%ld", len, send_res);
  return send_res;
}

int DataChannelHandler::SendBufferedMsg() {
  int ret = kSuccess;
  std::unique_lock<std::mutex> lock_guard(channel_mutex_);
  if (out_buffered_msg_.empty()) {
    return ret;
  }
  auto sid = out_buffered_msg_.front().first;
  auto msg = out_buffered_msg_.front().second;
  lock_guard.unlock();

  int send_res = SendInternal(sid, (const uint8_t*)msg.data(), msg.size());
  if (send_res < 0) {
    if (errno == EWOULDBLOCK) {
      tylog("sctp send block");
      send_blocking_ = true;
      ret = kSctpSendBlock;
    } else {
      tylog("sctp send failed, ret=%d, err=%s", send_res, strerror(errno));
      ret = kSctpSendError;
    }
  } else {
    lock_guard.lock();
    auto& front_msg = out_buffered_msg_.front().second;
    front_msg.erase(0, send_res);
    tylog("buffered msg size=%zu", front_msg.size());
    if (front_msg.empty()) {
      out_buffered_msg_.erase(out_buffered_msg_.begin());
      tylog("buffered partial msg sended");
      ret = kSuccess;
    } else {
      ret = kSctpSendBlock;
    }
  }

  return ret;
}

int DataChannelHandler::Send(const uint16_t sid, const uint8_t* buf,
                             const int len) {
  int ret = 0;
  std::unique_lock<std::mutex> lock_guard(channel_mutex_);
  if (!out_buffered_msg_.empty()) {
    tylog("have buffered msg, send will blocking, discard msg");
    send_blocking_ = true;
    return kSctpSendBlock;
  }
  lock_guard.unlock();

  int send_res = SendInternal(sid, buf, len);
  if (send_res < 0) {
    if (errno == EWOULDBLOCK) {
      tylog("sctp send block");
      send_blocking_ = true;
      ret = kSctpSendBlock;
    } else {
      tylog("sctp send failed, ret=%d, err=%s", send_res, strerror(errno));
      ret = kSctpSendError;
    }
  } else {
    if (send_res < len) {
      tylog("sctp partial send, len=%d, sent=%d", len, send_res);
      send_blocking_ = true;
      lock_guard.lock();
      out_buffered_msg_.push_back(std::make_pair(
          sid, std::string((const char*)buf + send_res, len - send_res)));
      ret = kSctpSendPartial;
    }
  }

  return ret;
}
