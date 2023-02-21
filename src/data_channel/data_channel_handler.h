#ifndef DATA_CHANNEL_DATA_CHANNEL_HANDLER_H_
#define DATA_CHANNEL_DATA_CHANNEL_HANDLER_H_

#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "usrsctplib/usrsctp.h"

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
  std::string label_;
  uint16_t sid_ = 0;

  uint8_t channel_type_ = 0;
  uint32_t reliability_params_ = 0;
  DataChannelStatus status_ = DataChannelStatusClosed;
  uint32_t lifetime = 0;
};

class SctpGlobalEnv {
 public:
  SctpGlobalEnv();
  ~SctpGlobalEnv();

  uintptr_t Register(DataChannelHandler* sctp);
  bool UnRegister(uintptr_t id);
  DataChannelHandler* Get(uintptr_t id);

  std::string ToString() const;

 private:
  uintptr_t id_;  // self increase

  std::mutex sctp_map_mutex_;
  std::map<uintptr_t, DataChannelHandler*> sctp_map_;
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
  int InitSocket();

  // active create
  int CreateDataChannel(const std::string& label);

  // handle received packet
  void HandleDataChannelPacket(const char* buf, const int nb_buf);

  // on recv
  int OnSctpEvent(const struct sctp_rcvinfo& rcv, void* data, size_t len);
  int OnSctpData(const struct sctp_rcvinfo& rcv, void* data, size_t len);

  // on send
  int SendBufferedMsg();

  // active send data
  int SendSctpDataForLable(const std::string& label,
                           const std::string& bufferToSend);
  // void OnSendThresholdCallback();
  // void OnSendSctpData(const uint8_t* data, size_t len);

 private:
  sctp_sendv_spa CreateSendParam(const DataChannel& data_channel);
  ssize_t SendInternal(const uint16_t sid, const std::string& bufferToSend);

  int SendSctpDataForSid(const uint16_t sid, const std::string& bufferToSend);

  int OnDataChannelControl(const struct sctp_rcvinfo& rcv, char* data, int len);
  int OnDataChannelMsg(const struct sctp_rcvinfo& rcv, char* data, int len);

 private:
  std::mutex channel_mutex_;
  std::map<uint16_t, DataChannel> data_channels_;
  std::map<std::string, uint16_t> label_sid_;
  std::vector<std::pair<uint16_t, std::string>> out_buffered_msg_;

  std::mutex sctp_mutex_;
  struct socket* sctp_socket_ = nullptr;

  std::atomic<bool> send_blocking_;

  // equal to SctpGlobalEnv id_ after register to global env
  uintptr_t id_ = 0;
  uint16_t data_channel_id_ = 0;  // self increase

  // tmp
 public:
  PeerConnection& belongingPeerConnection_;
};

#endif  // DATA_CHANNEL_DATA_CHANNEL_HANDLER_H_