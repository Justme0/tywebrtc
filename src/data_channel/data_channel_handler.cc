// SCTP doc https://datatracker.ietf.org/doc/html/rfc9260

#include "data_channel/data_channel_handler.h"

#include <fcntl.h>
#include <cassert>
#include <cstdarg>
#include <cstdlib>
#include <cstring>
#include <sstream>

#include "log/log.h"
#include "pc/peer_connection.h"

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

// DataMessageType is used for the SCTP "Payload Protocol Identifier", as
// defined in http://tools.ietf.org/html/rfc4960#section-14.4
//
// For the list of IANA approved values see:
// https://tools.ietf.org/html/rfc8831 Sec. 8
// http://www.iana.org/assignments/sctp-parameters/sctp-parameters.xml
// The value is not used by SCTP itself. It indicates the protocol running
// on top of SCTP.
// enum {
//     PPID_NONE = 0,  // No protocol is specified.
//     PPID_CONTROL = 50,
//     PPID_TEXT_LAST = 51,
//     PPID_BINARY_PARTIAL = 52,  // Deprecated
//     PPID_BINARY_LAST = 53,
//     PPID_TEXT_PARTIAL = 54,  // Deprecated
//     PPID_TEXT_EMPTY = 56,
//     PPID_BINARY_EMPTY = 57,
// };

enum DataChannelPPID {
  DataChannelPPIDControl = 50,
  DataChannelPPIDString = 51,
  DataChannelPPIDBinary = 53,
};

// uint16_t event_types[] = {SCTP_ADAPTATION_INDICATION, SCTP_ASSOC_CHANGE,
//                           SCTP_ASSOC_RESET_EVENT,     SCTP_REMOTE_ERROR,
//                           SCTP_SHUTDOWN_EVENT,        SCTP_SEND_FAILED_EVENT,
//                           SCTP_STREAM_RESET_EVENT, SCTP_STREAM_CHANGE_EVENT};

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

// This is the callback called from usrsctp when data has been received, after
// a packet has been interpreted and parsed by usrsctp and found to contain
// payload data. It is called by a usrsctp thread. It is assumed this function
// will free the memory used by 'data'.
static int RecvSctpDataCallback(struct socket* sock,
                                union sctp_sockstore /*addr*/, void* data,
                                size_t len, struct sctp_rcvinfo rcv, int flags,
                                void* ulp_info) {
  DataChannelHandler* sctp = GetSctpFromSocket(sock);

  if (sctp == nullptr) {
    tylog("no found sctp");
    if (data) {
      free(data);  // to use RAII
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
    sctp->OnSctpEvent(rcv, std::string(static_cast<const char*>(data), len));
  } else {
    sctp->OnSctpData(rcv, std::string(static_cast<const char*>(data), len));
  }
  if (data) {
    free(data);
  }
  return kSctpSuccess;
}

// as create socket `usrsctp_socket()` callback param.
// Fired on our I/O thread. UsrsctpTransport::OnPacketReceived() gets
// a packet containing acknowledgments, which goes into usrsctp_conninput,
// and then back here.
static int SendThresholdCallback(struct socket* sock, uint32_t sb_free,
                                 void* /*ulp_info*/) {
  int ret = 0;
  DataChannelHandler* sctp = GetSctpFromSocket(sock);

  if (sctp == nullptr) {
    tylog("no found sctp");
    return -1;
  }

  tylog("sb_free=%u", sb_free);

  ret = sctp->SendBufferedMsg();
  if (ret) {
    tylog("sendBufferedMsg ret=%d.", ret);

    return ret;
  }

  return 0;
}

static int SendSctpDataCallback(void* addr, void* data, size_t len,
                                uint8_t /*tos*/, uint8_t /*set_df*/) {
  tylog("callback send write data=%p, len=%zu.", data, len);

  DataChannelHandler* sctp = g_sctp_env->Get(reinterpret_cast<uintptr_t>(addr));

  if (sctp == nullptr) {
    tylog("no found sctp addr=%p, sctpEnvMap=%s.", addr,
          g_sctp_env->ToString().data());

    return -1;
  }

  sctp->belongingPeerConnection_.dtlsHandler_.SendToDtls(data, len);

  return 0;
}

SctpGlobalEnv::SctpGlobalEnv() : id_(0) {
  tylog("sctp global env init");
  usrsctp_init(0, SendSctpDataCallback, SctpDebugLog);

  usrsctp_sysctl_set_sctp_ecn_enable(0);
  usrsctp_sysctl_set_sctp_asconf_enable(0);
  usrsctp_sysctl_set_sctp_auth_enable(0);
  usrsctp_sysctl_set_sctp_nr_outgoing_streams_default(kMaxStream);

  // usrsctp_sysctl_set_sctp_debug_on(SCTP_DEBUG_ALL);
}

std::string SctpGlobalEnv::ToString() const {
  return tylib::format_string("{id=%lu, map=%s}", id_,
                              tylib::AnyToString(sctp_map_).data());
}

SctpGlobalEnv::~SctpGlobalEnv() { usrsctp_finish(); }

//
uintptr_t SctpGlobalEnv::Register(DataChannelHandler* sctp) {
  std::unique_lock<std::mutex> lock_guard(sctp_map_mutex_);

  auto id = ++id_;

  sctp_map_[id] = sctp;

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
    : belongingPeerConnection_(pc) {}

DataChannelHandler::~DataChannelHandler() {
  tylog("destroy datachannel sctp_socket_=%p, id=%lu.", sctp_socket_, id_);

  std::unique_lock<std::mutex> lock_guard(sctp_mutex_);
  if (sctp_socket_) {
    usrsctp_close(sctp_socket_);
    usrsctp_deregister_address(reinterpret_cast<void*>(id_));
    g_sctp_env->UnRegister(id_);
  }
}

// NOTE, check if init already, but not best
int DataChannelHandler::InitSocket() {
  if (nullptr != this->sctp_socket_) {
    tylog("already init sctp socket=%p, no need again", sctp_socket_);

    return 0;
  }

  int ret = 0;

  if (g_sctp_env == nullptr) {
    g_sctp_env = new SctpGlobalEnv();
  }

  id_ = g_sctp_env->Register(this);

  data_channel_id_ = 1;

  send_blocking_ = false;

  tylog("sctp id %lu, register it", id_);

  usrsctp_register_address(reinterpret_cast<void*>(id_));

  // ref: https://github.com/sctplab/usrsctp/blob/master/Manual.md
  sctp_socket_ =
      usrsctp_socket(AF_CONN, SOCK_STREAM, IPPROTO_SCTP, RecvSctpDataCallback,
                     SendThresholdCallback, 0, this);
  if (nullptr == sctp_socket_) {
    tylog("create sctp socket fail, errno=%d[%s]", errno, strerror(errno));

    return -1;
  }

  ret = usrsctp_set_non_blocking(sctp_socket_, 1);
  if (ret < 0) {
    tylog("usrrsctp set non blocking failed, ret=%d, err=%d(%s)", ret, errno,
          strerror(errno));

    return ret;
  }

  struct sctp_assoc_value av;

  av.assoc_value = SCTP_ENABLE_RESET_STREAM_REQ | SCTP_ENABLE_RESET_ASSOC_REQ |
                   SCTP_ENABLE_CHANGE_ASSOC_REQ;
  // ref:
  // https://github.com/sctplab/usrsctp/blob/master/Manual.md#socket-options
  ret = usrsctp_setsockopt(sctp_socket_, IPPROTO_SCTP, SCTP_ENABLE_STREAM_RESET,
                           &av, sizeof(av));
  if (ret) {
    tylog("usrrsctp set SCTP_ENABLE_STREAM_RESET failed, ret=%d, err=%d(%s)",
          ret, errno, strerror(errno));

    return ret;
  }

  uint32_t no_delay = 1;
  ret = usrsctp_setsockopt(sctp_socket_, IPPROTO_SCTP, SCTP_NODELAY, &no_delay,
                           sizeof(no_delay));
  if (ret) {
    tylog("usrrsctp set SCTP_NODELAY failed, ret=%d, err=%d(%s)", ret, errno,
          strerror(errno));
    return ret;
  }

  uint32_t eor = 1;
  ret = usrsctp_setsockopt(sctp_socket_, IPPROTO_SCTP, SCTP_EXPLICIT_EOR, &eor,
                           sizeof(eor));

  if (ret) {
    tylog("usrrsctp set SCTP_EXPLICIT_EOR failed, ret=%d, err=%d(%s)", ret,
          errno, strerror(errno));

    return ret;
  }

  // Subscribe to SCTP event notifications.
  // TODO(crbug.com/1137936): Subscribe to SCTP_SEND_FAILED_EVENT once deadlock
  // is fixed upstream, or we switch to the upcall API:
  // https://github.com/sctplab/usrsctp/issues/537
  int event_types[] = {SCTP_ASSOC_CHANGE, SCTP_PEER_ADDR_CHANGE,
                       SCTP_SENDER_DRY_EVENT, SCTP_STREAM_RESET_EVENT};
  struct sctp_event event;
  memset(&event, 0, sizeof event);
  event.se_assoc_id = SCTP_ALL_ASSOC;
  event.se_on = 1;

  for (size_t i = 0; i < sizeof(event_types) / sizeof(*event_types); ++i) {
    event.se_type = event_types[i];
    ret = usrsctp_setsockopt(sctp_socket_, IPPROTO_SCTP, SCTP_EVENT, &event,
                             sizeof(event));
    if (ret) {
      tylog("usrrsctp set SCTP_NODELAY failed, ret=%d, err=%d(%s)", ret, errno,
            strerror(errno));

      return ret;
    }
  }

  // Init message.
  struct sctp_initmsg initmsg;
  memset(&initmsg, 0, sizeof(initmsg));
  initmsg.sinit_num_ostreams = kMaxStream;
  initmsg.sinit_max_instreams = kMaxStream;

  ret = usrsctp_setsockopt(sctp_socket_, IPPROTO_SCTP, SCTP_INITMSG, &initmsg,
                           sizeof(initmsg));
  if (ret) {
    tylog("usrrsctp set SCTP_INITMSG failed, ret=%d, err=%d(%s)", ret, errno,
          strerror(errno));

    return ret;
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

  // To connect

  struct sockaddr_conn rconn;
  memset(&rconn, 0, sizeof(rconn));
  rconn.sconn_family = AF_CONN;
  rconn.sconn_port = htons(kSctpPort);
  rconn.sconn_addr = reinterpret_cast<void*>(id_);

  ret = usrsctp_connect(
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
  if (ret) {
    tylog("usrrsctp set SCTP_PEER_ADDR_PARAMS failed, ret=%d, err=%d(%s)", ret,
          errno, strerror(errno));

    return ret;
  }

  tylog("usrsctp connect peer success.");

  return 0;
}

// Passive received packet to SCTP stack. Once processed by usrsctp, the data
// will be will be given to the global OnSctpInboundData, and then, marshalled
// by the AsyncInvoker.
// then SendThresholdCallback()
void DataChannelHandler::HandleDataChannelPacket(const char* buf,
                                                 const int nb_buf) {
  assert(nullptr != this->sctp_socket_);

  usrsctp_conninput(reinterpret_cast<void*>(id_), buf, nb_buf, 0);
}

int DataChannelHandler::CreateDataChannel(const std::string& label) {
  DataChannel data_channel;
  data_channel.label_ = label;
  data_channel.sid_ = data_channel_id_++;
  data_channel.channel_type_ = DataChannelChannelTypeReliable;
  data_channel.reliability_params_ = 0;
  data_channel.status_ = DataChannelStatusOpen;

  label_sid_[data_channel.label_] = data_channel.sid_;
  data_channels_[data_channel.sid_] = data_channel;

  int reqlen = sizeof(struct DcepOpenMessage) + label.size();

  // OPT: not use calloc
  DcepOpenMessage* req = (struct DcepOpenMessage*)calloc(1, reqlen);
  req->message_type = DataChannelMessageTypeOpen;
  req->channel_type = DataChannelChannelTypeReliable;
  req->priority = htons(0);                     /* XXX: add support */
  req->reliability_params = htons((uint16_t)0); /* XXX Why 16-bit */
  if (!label.empty()) {
    req->label_length = htons(label.size());
    memcpy(req->label_and_protocol, label.c_str(), label.size());
  }

  sctp_sndinfo sndinfo;
  memset(&sndinfo, 0, sizeof(struct sctp_sndinfo));
  sndinfo.snd_sid = data_channel.sid_;
  sndinfo.snd_flags = SCTP_EOR;
  sndinfo.snd_ppid = htonl(DataChannelPPIDControl);

  ssize_t send_res =
      usrsctp_sendv(sctp_socket_, req, reqlen, nullptr, 0, &sndinfo,
                    sizeof sndinfo, SCTP_SENDV_SNDINFO, 0);
  if (send_res == -1) {
    tylog("sctp_sendv open request failed, errno=%d[%s]", errno,
          strerror(errno));
    free(req);

    return -1;
  }

  // what if send partial ?
  assert(send_res == reqlen);

  tylog("sctp_sendv open request succ.");
  free(req);

  return 0;
}

const std::string SctpNotifyToString(uint16_t notifyType) {
  switch (notifyType) {
    case SCTP_ASSOC_CHANGE:
      return "SCTP_ASSOC_CHANGE";
    case SCTP_PEER_ADDR_CHANGE:
      return "SCTP_PEER_ADDR_CHANGE";
    case SCTP_REMOTE_ERROR:
      return "SCTP_REMOTE_ERROR";
    case SCTP_SEND_FAILED:
      return "SCTP_SEND_FAILED";
    case SCTP_SHUTDOWN_EVENT:
      return "SCTP_SHUTDOWN_EVENT";
    case SCTP_ADAPTATION_INDICATION:
      return "SCTP_ADAPTATION_INDICATION";
    case SCTP_PARTIAL_DELIVERY_EVENT:
      return "SCTP_PARTIAL_DELIVERY_EVENT";
    case SCTP_AUTHENTICATION_EVENT:
      return "SCTP_AUTHENTICATION_EVENT";
    case SCTP_STREAM_RESET_EVENT:
      return "SCTP_STREAM_RESET_EVENT";
    case SCTP_SENDER_DRY_EVENT:
      return "SCTP_SENDER_DRY_EVENT";
    case SCTP_NOTIFICATIONS_STOPPED_EVENT:
      return "SCTP_NOTIFICATIONS_STOPPED_EVENT";
    case SCTP_ASSOC_RESET_EVENT:
      return "SCTP_ASSOC_RESET_EVENT";
    case SCTP_STREAM_CHANGE_EVENT:
      return "SCTP_STREAM_CHANGE_EVENT";
    case SCTP_SEND_FAILED_EVENT:
      return "SCTP_SEND_FAILED_EVENT";
    default:
      return tylib::format_string("UnknownNotifyType[%d]", notifyType);
  }
}

int DataChannelHandler::OnSctpEvent(const struct sctp_rcvinfo&,
                                    const std::string& receivedBuffer) {
  const union sctp_notification* sctp_notify =
      reinterpret_cast<const union sctp_notification*>(receivedBuffer.data());
  if (sctp_notify->sn_header.sn_length != receivedBuffer.size()) {
    tylog("sctp notify header");

    return -1;
  }

  tylog("recv sctp event type=%s.",
        SctpNotifyToString(sctp_notify->sn_header.sn_type).data());

  // notify is union, handle according to sn_type
  switch (sctp_notify->sn_header.sn_type) {
    // should handle other enum ?
    case SCTP_SEND_FAILED_EVENT: {
      const sctp_send_failed_event& ssfe = sctp_notify->sn_send_failed_event;
      tylog("SCTP_SEND_FAILED_EVENT, ppid=%u, sid=%u, flags=0x%04x, err=0x%08x",
            ntohl(ssfe.ssfe_info.snd_ppid), ssfe.ssfe_info.snd_sid,
            ssfe.ssfe_info.snd_flags, ssfe.ssfe_error);
      break;
    }

    default:
      break;
  }

  return kSctpSuccess;
}

int DataChannelHandler::OnSctpData(const struct sctp_rcvinfo& rcv,
                                   const std::string& receivedBuffer) {
  int err = 0;

  uint32_t ppid = ntohl(rcv.rcv_ppid);
  switch (ppid) {
    case DataChannelPPIDControl:
      err = OnDataChannelControl(rcv, receivedBuffer);
      break;
    case DataChannelPPIDString:
    case DataChannelPPIDBinary:
      err = OnDataChannelMsg(rcv, receivedBuffer);
      break;
    default:
      break;
  }
  return err;
}

int DataChannelHandler::OnDataChannelControl(
    const struct sctp_rcvinfo& rcv, const std::string& receivedBuffer) {
  if (receivedBuffer.size() <= 12) {
    tylog("sctp data length invalid");
    return -1;
  }

  int pos = 0;
  uint32_t byte_left = receivedBuffer.size();
  const char* data = receivedBuffer.data();
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

        default:
          tylog("unknown channel_type=%d", channel_type);
          break;
      }

      std::string label;
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
          "key: channel_type=%u, priority=%u, reliability_params=%u, "
          "label_length=%u, "
          "protocol_length=%u, label=%s, rcv_sid=%d.",
          channel_type, priority, reliability_params, label_length,
          protocol_length, label.c_str(), rcv.rcv_sid);

      DataChannel data_channel;
      data_channel.label_ = label;
      data_channel.sid_ = rcv.rcv_sid;
      data_channel.channel_type_ = channel_type;
      data_channel.reliability_params_ = reliability_params;
      data_channel.status_ = DataChannelStatusOpen;

      label_sid_[data_channel.label_] = data_channel.sid_;
      data_channels_[data_channel.sid_] = data_channel;

      // may callback OnDataChannelOpen

      break;
    }
    case DataChannelMessageTypeAck: {
      break;
    }
    default: {
      tylog("unknown data channel control msg type=%d", msg_type);

      assert(!"unknown data channel control msg type, shouldn't use assert :)");
    }
  }

  return 0;
}

int DataChannelHandler::OnDataChannelMsg(const struct sctp_rcvinfo& rcv,
                                         const std::string& receivedBuffer) {
  int ret = 0;

  std::unique_lock<std::mutex> lock_guard(channel_mutex_);

  std::map<uint16_t, DataChannel>::iterator iter =
      data_channels_.find(rcv.rcv_sid);
  if (iter == data_channels_.end()) {
    tylog("can not found sid=%d", rcv.rcv_sid);
    return -1;
  }
  const std::string label = iter->second.label_;

  lock_guard.unlock();

  // raw data may have '\0', use .*
  tylog("on recv data len=%zu, data=%p %.*s.", receivedBuffer.size(),
        receivedBuffer.data(), static_cast<int>(receivedBuffer.size()),
        receivedBuffer.data());

  // to use string view
  auto i = receivedBuffer.find("g_vp8Payload=");
  size_t size = strlen("g_vp8Payload=");
  if (i != std::string::npos) {
    assert(i == 0);
    int payload = atoi(receivedBuffer.substr(size).data());
    tylog("taylor payload=%d.", payload);
    this->belongingPeerConnection_.sdpHandler_.vp8PayloadType = payload;

    // not send to peer
    return 0;
  }

  auto peerPC = belongingPeerConnection_.FindPeerPC();
  if (nullptr == peerPC) {
    return 0;
  }

  // OPT: use string_view
  ret = peerPC->dataChannelHandler_.SendSctpDataForLable(receivedBuffer);
  if (ret) {
    tylog("sendSctpDataForLable ret=%d.", ret);

    return ret;
  }

  return 0;
}

// key
int DataChannelHandler::SendSctpDataForLable(const std::string& bufferToSend) {
  const std::string& label = "sendChannel";
  uint16_t sid = 0;
  std::unique_lock<std::mutex> lock_guard(channel_mutex_);

  std::map<std::string, uint16_t>::iterator iter = label_sid_.find(label);
  if (iter == label_sid_.end()) {
    tylog("can not found label=%s", label.c_str());
    return kChannelNotFound;
  }
  sid = iter->second;

  lock_guard.unlock();

  return SendSctpDataForSid(sid, bufferToSend);
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

// NOTE: return is same as usrsctp_sendv.
// returns the number of bytes sent, or -1 if an error occurred. The variable
// errno is then set appropriately.
ssize_t DataChannelHandler::SendInternal(const uint16_t sid,
                                         const std::string& bufferToSend) {
  // DataChannelHandler* sctp = GetSctpFromSocket(sock);

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

  // already connected, so no need to specify dst address
  // https://github.com/sctplab/usrsctp/blob/master/Manual.md#usrsctp_sendv
  ssize_t send_res =
      usrsctp_sendv(sctp_socket_, bufferToSend.data(), bufferToSend.size(),
                    nullptr, 0, &spa, sizeof spa, SCTP_SENDV_SPA, 0);
  if (-1 == send_res) {
    tylog("sctp sendv return -1, errno=%d[%s].", errno, strerror(errno));
  } else if (send_res != static_cast<int>(bufferToSend.size())) {
    tylog("send partial, all len=%zu, only send=%ld.", bufferToSend.size(),
          send_res);
  }

  return send_res;
}

// send the first one in buffered queue
int DataChannelHandler::SendBufferedMsg() {
  if (!send_blocking_) {
    return 0;
  }

  std::unique_lock<std::mutex> lock_guard(channel_mutex_);
  if (out_buffered_msg_.empty()) {
    tylog("outbound buf queue empty, no need send");

    send_blocking_ = false;

    return 0;
  }

  auto sid = out_buffered_msg_.front().first;
  auto msg = out_buffered_msg_.front().second;
  lock_guard.unlock();

  ssize_t send_res = SendInternal(sid, msg);
  if (send_res < 0) {
    if (errno == EWOULDBLOCK) {
      tylog("sctp send block");
      send_blocking_ = true;

      return kSctpSendBlock;
    }

    tylog("sctp send failed, ret=%ld, err=%s", send_res, strerror(errno));

    return kSctpSendError;
  }

  lock_guard.lock();

  assert(send_res <= static_cast<int>(msg.size()));
  if (send_res != static_cast<int>(msg.size())) {
    tylog("send partial data, all=%zu, send=%ld, remaining=%ld.", msg.size(),
          send_res, msg.size() - send_res);
    msg.erase(0, send_res);

    return kSctpSendBlock;
  }

  tylog("first buffered msg all sended");
  out_buffered_msg_.erase(out_buffered_msg_.begin());

  send_blocking_ = false;
  return 0;
}

// only called by SendSctpDataForLable
int DataChannelHandler::SendSctpDataForSid(const uint16_t sid,
                                           const std::string& bufferToSend) {
  std::unique_lock<std::mutex> lock_guard(channel_mutex_);
  if (!out_buffered_msg_.empty()) {
    tylog("have buffered msg, send will blocking, discard msg");
    send_blocking_ = true;

    return kSctpSendBlock;
  }
  lock_guard.unlock();

  tylog("taylor send=%s.", bufferToSend.data());
  ssize_t send_res = SendInternal(sid, bufferToSend);
  if (send_res < 0) {
    if (errno == EWOULDBLOCK) {
      tylog("sctp send block");
      send_blocking_ = true;
      return kSctpSendBlock;
    }

    tylog("sctp send failed, ret=%ld, errno=%d[%s]", send_res, errno,
          strerror(errno));

    return kSctpSendError;
  }

  if (send_res < static_cast<int>(bufferToSend.size())) {
    tylog("sctp partial send, len=%zu, sent=%ld", bufferToSend.size(),
          send_res);
    send_blocking_ = true;
    lock_guard.lock();
    out_buffered_msg_.push_back(
        std::make_pair(sid, std::string(bufferToSend.data() + send_res,
                                        bufferToSend.size() - send_res)));

    return kSctpSendPartial;
  }

  return 0;
}
