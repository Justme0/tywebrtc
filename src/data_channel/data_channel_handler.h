#ifndef DATA_CHANNEL_DATA_CHANNEL_HANDLER_H_
#define DATA_CHANNEL_DATA_CHANNEL_HANDLER_H_

#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "usrsctplib/usrsctp.h"
// #include "api/webrtc_core_interface.h"

class PeerConnection;
class DataChannelHandler;

// sctp error code
const int kSuccess = 0;
const int kSctpSendError = -10;
const int kSctpSendPartial = -11;
const int kSctpSendBlock = -12;

// datachannel error code
const int kChannelNotFound = -20;
const int kChannelError = -21;

enum DataChannelStatus {
  DataChannelStatusClosed = 1,
  DataChannelStatusOpen = 2,
};

#define DATA_CHANNEL_RELIABLE 0x00

struct DcepOpenMessage {
  uint8_t message_type;
  uint8_t channel_type;
  uint16_t priority;
  uint32_t reliability_params;
  uint16_t label_length;
  uint16_t protocol_length;
  char label_and_protocol[0];
} __attribute__((packed, aligned(1)));

struct DcepAckMessage {
  uint8_t message_type;
} __attribute__((packed, aligned(1)));

struct DataChannel {
  DataChannel();

  std::string label_;
  uint16_t sid_;

  uint8_t channel_type_;
  uint32_t reliability_params_;
  DataChannelStatus status_;
  uint32_t lifetime;
};

class SctpGlobalEnv {
 public:
  SctpGlobalEnv();
  ~SctpGlobalEnv();

 public:
  uintptr_t Register(DataChannelHandler* sctp);
  DataChannelHandler* Get(uintptr_t id);
  bool UnRegister(uintptr_t id);

 private:
  std::mutex sctp_map_mutex_;
  std::map<uintptr_t, DataChannelHandler*> sctp_map_;
  uintptr_t id_;
};

class DataChannelHandler {
 public:
  // class Observer {
  //  public:
  //   Observer() = default;
  //   virtual ~Observer() = default;
  //   virtual void OnRecvDataChannelMsg(const std::string& label,
  //                                     std::unique_ptr<Packet> data) = 0;
  //   virtual void OnDataChannelOpen(const std::string& label) = 0;
  //   virtual void OnSendToDtlsChannel(char* data, size_t len) = 0;
  // };

 public:
  // DataChannelHandler(Observer* obs, const std::string& stream_id);
  explicit DataChannelHandler(PeerConnection& pc);

  DataChannelHandler(const DataChannelHandler&) = delete;
  DataChannelHandler& operator=(const DataChannelHandler&) = delete;
  ~DataChannelHandler();

 public:
  int ConnectToClass();
  int Send(const std::string& label, const uint8_t* buf, const int len);
  void OnSendThresholdCallback();
  void OnSendSctpData(const uint8_t* data, size_t len);

  int OnSctpEvent(const struct sctp_rcvinfo& rcv, void* data, size_t len);
  int OnSctpData(const struct sctp_rcvinfo& rcv, void* data, size_t len);

  void Feed(const uint8_t* buf, const int nb_buf);

  const std::string& GetStreamId() const { return stream_id_; }
  int CreateDataChannel(const std::string& label);

 private:
  sctp_sendv_spa CreateSendParam(const DataChannel& data_channel);
  int SendBufferedMsg();
  int SendInternal(const uint16_t sid, const uint8_t* buf, const int len);

  int Send(const uint16_t sid, const uint8_t* buf, const int len);

  int OnDataChannelControl(const struct sctp_rcvinfo& rcv, char* data, int len);
  int OnDataChannelMsg(const struct sctp_rcvinfo& rcv, char* data, int len);

 private:
  std::mutex channel_mutex_;
  std::string stream_id_;
  std::map<uint16_t, DataChannel> data_channels_;
  std::map<std::string, uint16_t> label_sid_;
  std::vector<std::pair<uint16_t, std::string> > out_buffered_msg_;

  std::atomic<bool> send_blocking_;
  uintptr_t id_;

  uint16_t data_channel_id_;

  std::mutex sctp_mutex_;
  struct socket* sctp_socket_;

  PeerConnection& belongingPeerConnection_;
};

#endif  // DATA_CHANNEL_DATA_CHANNEL_HANDLER_H_