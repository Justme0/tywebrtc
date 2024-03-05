#ifndef RTMP_RTMP_PULL_H_
#define RTMP_RTMP_PULL_H_

// #include <sys/queue.h>

#include <cstring>
#include <map>
#include <string>

#include "librtmp/log.h"
#include "librtmp/rtmp_sys.h"

#include "pull/pull_handler.h"
#include "rtmp/FlvAssist.h"
#include "rtmp/MediaBuffer.h"
#include "rtp/rtp_parser.h"

#define MAX_CLIENT (256)  //单进程最大支持128路连接

//一路UDP的监听socket，用于和接口机适配进程通信，一个TCP监听socket，用于TCP监听。
#define START_CLIENT_IDX (2)

#define MAX_SLOT (MAX_CLIENT + START_CLIENT_IDX)
#define MAX_SPS_PPS_LEN (2048)
#define MAX_RTMP_TRUNK_SIZE (1360)
#define GOD_ID (108688)

#define VIDEO_RAW_STM_MAX_LEN (2048000)
#define AUDIO_RAW_STM_MAX_LEN (4096)
#define UDP_RECV_BUFF_MAX_LEN (16384)

#define MAX_DOMAIN_NUM (3)
#define MAX_DOMAIN_ALIVE_TIME (180000)
#define MAX_PKG_LEN (2048)
#define MAX_PROCESS_TIME (100)

#define MAX_RTP_PKG_SIZE (1300)

#define VIDEO_MIN_BITRATE (200)

#define VIDEO_MAX_BITRATE (16000)

#define DEFAULT_AUDIO_RTP_TS_INTERVAL (2400)

#define RTMPIDX(r) ((r) - &m_Contexts[0])
#define CLIENTIDX(c) ((c) - &m_Clients[0])

#define AVC_P(str) \
  { const_cast<char*>(str), sizeof(str) - 1 }
#define SAVC(x) static const AVal av_##x = AVC_P(#x)
#define STR2AVAL(av, str) \
  av.av_val = (char*)str; \
  av.av_len = strlen(av.av_val)

SAVC(connect);
SAVC(createStream);
SAVC(releaseStream);
SAVC(closeStream);
SAVC(publish);
SAVC(_result);
SAVC(_error);
SAVC(error);
SAVC(fmsVer);
SAVC(capabilities);
SAVC(mode);
SAVC(level);
SAVC(code);
SAVC(description);
SAVC(objectEncoding);
SAVC(onStatus);
SAVC(details);
SAVC(clientid);
SAVC(play);
SAVC(onMetaData);
SAVC(kTXRTMP_METADATA_PROXY_MSG);
SAVC(TENCENT_MSG_RTMP_PB);
SAVC(encoder);
SAVC(video);
SAVC(audio);
SAVC(duration);
SAVC(onFCPublish);
SAVC(FCPublish);
SAVC(app);
SAVC(videoCodecid);
SAVC(fileSize);
SAVC(width);
SAVC(height);
SAVC(framerate);
SAVC(FCUnpublish);
SAVC(videodatarate);

typedef enum enRtmpClientType {
  RTMP_CLIENT_MINI_PROGRAM = 1,
  RTMP_CLIENT_THIRD = 2,
  RTMP_CLIENT_RTMP_SRV = 3,
  RTMP_CLIENT_BUTT
} RTMP_CLIENT_TYPE;

enum RTMPTimeCostType { RTMP_TIME_COST_CLIENT, RTMP_TIME_COST_ACC };

enum RTMPChannel {
  RTMP_NETWORK_CHANNEL = 2,  ///< channel for network-related messages
                             ///(bandwidth report, ping, etc)
  RTMP_SYSTEM_CHANNEL,       ///< channel for sending server control messages
  RTMP_AUDIO_CHANNEL,        ///< channel for audio data
  RTMP_VIDEO_CHANNEL = 6,    ///< channel for video data
  RTMP_SOURCE_CHANNEL = 8,   ///< channel for a/v invokes
};

enum Mpeg2AacSample {
  e_AacSample_96000Hz = 0,
  e_AacSample_88200Hz = 1,
  e_AacSample_64000Hz = 2,
  e_AacSample_48000Hz = 3,
  e_AacSample_44100Hz = 4,
  e_AacSample_32000Hz = 5,
  e_AacSample_24000Hz = 6,
  e_AacSample_22050Hz = 7,
  e_AacSample_11025Hz = 8,
  e_AacSample_8000Hz = 9,
  e_AacSample_7350Hz = 10
};

enum {
  STREAMING_ACCEPTING,
  STREAMING_HANDSHAKE,
  STREAMING_IN_PROGRESS,
  STREAMING_STOPPING,
  STREAMING_STOPPED
};

typedef enum enConnectiedMod {
  CONNECTED_MODE_DIRECT = 1,
  CONNECTED_MODE_COMMON_DOMAIN = 2,
  CONNECTED_MODE_TEST = 3,

  CONNECTED_MODE_BUTT
} CONNECTED_MODE;

#pragma pack(1)

typedef struct TagFlvAvcHead {
  char cfgVersion;

  char avcProfile;

  char profileCompatibility;

  char avcLevel;

  char lengthSizeMinusOne : 2;

  char reserved1 : 6;

  char numOfSPS : 5;

  char reserved2 : 3;

} FlvAvcHead;

typedef struct AudTagHead {
  char SoundType : 1;
  char SoundSize : 1;
  char SoundRate : 2;
  char AudFormat : 4;
  char AACPackType;
} AUD_TAG_HEAD;

typedef struct VidTagHead {
  char CodecID : 4;
  char Frametype : 4;
  char AVCPacketType : 8;

} VID_TAG_HEAD;

/*

 0					 1 2
3

 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1

+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

|	   'O'		|	   'p'		|	   'u'
|	   's'		|

+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

|	   'H'		|	   'e'		|	   'a'
|	   'd'		|

+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

|  Version = 1	| Channel Count |			Pre-skip
|

+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

|					  Input Sample Rate (Hz)
|

+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

|	Output Gain (Q7.8 in dB)	| Mapping Family|
|

+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ :

|
|

:				Optional Channel Mapping Table...
:

|
|

+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

*/

// rfc7845

typedef struct AudOpusHead

{
  unsigned char OpusName[8];
  unsigned char Ver;
  unsigned char ChannelCount;
  unsigned short Preskip;
  unsigned int InputSampleRate;
  unsigned short OutputGain;
  unsigned char MappingFamily;

} AUD_OPUS_HEAD;

// | audioObjectType(5)[FlvAacProfile] |
// samplingFrequencyIndex(4)[Mpeg2AacSample] | channelConfiguration(4) |
// frameLengthFlag(1) | dependsOnCoreCoder(1) | extensionFlag(1) |
typedef struct AudAACSeqHead {
  unsigned char SampleFrIdxH : 3;
  unsigned char AudObjType : 5;
  unsigned char ExtFlag : 1;
  unsigned char DeponCoreCoder : 1;
  unsigned char FrLenFlag : 1;
  unsigned char ChnCfg : 4;
  unsigned char SampleFrIdxL : 1;

} AUD_AAC_SEQ_HEAD;

typedef struct TagAdtsHead {
  unsigned int syncwordH : 8;  // 0xFF

  unsigned int absent : 1;
  unsigned int layer : 2;
  unsigned int ID : 1;
  unsigned int syncwordL : 4;

  unsigned int ChannelNumH : 1;
  unsigned int private_bit : 1;
  unsigned int Sample : 4;
  unsigned int profile : 2;

  unsigned int frame_lengthH : 2;
  unsigned int copyright_start : 1;
  unsigned int copyright_bit : 1;
  unsigned int home : 1;
  unsigned int original_copy : 1;
  unsigned int ChannelNumL : 2;

  unsigned int frame_lengthM : 8;

  unsigned int fullnessH : 5;
  unsigned int frame_lengthL : 3;

  unsigned int blocks : 2;
  unsigned int fullnessL : 6;
} ADTS_HEAD;

#define DATA_STX 0x25

#define DATA_ETX 0x26

typedef struct RtmpDataHead

{
  unsigned char Start;

  unsigned long long SrcID;

  unsigned int TimeStamp;

  unsigned char FrameType;  // WEB_VIDEO_FRAME_TYPE

  unsigned char MediaType;  // MEDIA_TYPE
  unsigned long long SessionID;

  unsigned char Reverse[7];

} RTMP_DATA_HEAD;

#pragma pack()

typedef enum enRtmpState {
  RTMP_STATE_INIT,
  RTMP_STATE_TCP_CONNECTED,
  RTMP_STATE_RTMP_CONNECTED,
  RTMP_STATE_PLAY,
  RTMP_STATE_PUBLISH,
} RTMP_STATE;

typedef enum enRtmpDisConnectReason {
  RTMP_DISCONNECT_SOCKET = 1,
  RTMP_DISCONNECT_TIMEOUT = 2,
  RTMP_DISCONNECT_UNPUBLISH = 3,
  RTMP_DISCONNECT_NORMAL = 4,
  RTMP_DISCONNECT_BUTT

} RTMP_DISCONNECT_REASON;

struct Client {
  Client();

  FILE* pfOutfpH264 = nullptr;
  FILE* pfOutfpAAC = nullptr;
  FILE* pfOutfpFLV = nullptr;
  char DumpFlag = 0;  // 是否已开始dump
  char State;         // RTMP_STATE
  unsigned long long SessionID = 0;
  unsigned long long TinyId = 0;
  unsigned long long SrcTinyId;
  ADTS_HEAD AdtsHead;
  unsigned long long EnterTime = 0;
  unsigned long long ActiveTime = 0;
  unsigned long long HeartTime = 0;
  unsigned long long RecvAccTime = 0;
  unsigned char pPps[MAX_SPS_PPS_LEN];
  unsigned int PpsLen = 0;
  unsigned char pSps[MAX_SPS_PPS_LEN];
  unsigned int SpsLen = 0;
  unsigned char PpsNum = 0;

  unsigned char SpsNum = 0;

  char IFlag = 0;
  unsigned int RecvAccVidNum;
  unsigned int RecvAccAudNum;
  unsigned int LstRecvAccAudTs;
  unsigned int LstRecvAccVidTs;
  unsigned int RecvRtmpVidNum;
  unsigned int RecvRtmpAudNum;
  unsigned int LstRecvRtmpAudTs;
  unsigned int LstRecvRtmpVidTs;
  unsigned char AudFormat;  // e_FlvAudioSoundFormat_None
  unsigned int VideoFrCycle;
  unsigned int VideoBitCycle;
  unsigned int AudioBitCycle;
  unsigned int IFrameNum;
  unsigned int SpsPpsNum;
  int VideoWidth;
  int VideoHeight;
  unsigned int ClientIp;
  unsigned int MsgToClientNum;
  unsigned int MsgToAccNum;

  //封装RTP数据 注意观看列表用户发生变化的时候需要重新初始化
  // VideoRfc3984* pVideoRtpPack;

  unsigned char ReqIFrFlag;
  unsigned int ReqIFrCnt;
  unsigned long long LstReqIFrTimeMs;
  void* pRtmpSrv;
  unsigned char IFecCfg;
  unsigned char PFecCfg;
  unsigned int DropSeiNum;
  unsigned int RecvBframeNum;

  unsigned int PassSeiNum;
  int ClientType;  // RTMP_CLIENT_BUTT
  unsigned short WebrtcCLientPort;
  std::string StmID;
  unsigned int Seq;
  unsigned int AudSeq;
  std::string Url;
  std::string BakUrl;
  std::string TcUrl;
  int ReConnectTime;
  int ReConnectFlag;
  unsigned int ReConnectIntervalMs;
  unsigned long long StartConnectTime;
  unsigned long long HandShakeOkTime;
  unsigned long long RecvFirstVidPkgTime;
  unsigned long long SendFirstVidPkgTime;
  unsigned int CfgVideoBitRate;

  unsigned int IsSever;

  unsigned int RtmpAudPkgNumLastCycle;

  unsigned int RtmpVidTsLastCycle;

  unsigned int GopLength;

  unsigned long long LastRecvIframeTime;

  unsigned int RtmpVidTsPass;

  unsigned int AudFr;

  unsigned char AudChn;

  unsigned int AudSample;

  unsigned short AudFrLen;

  unsigned int AudioRtpTs;

  unsigned short AudioRtpTsInterVal;

  unsigned int IsSeiPass;

  // LIST_ENTRY(client) next;
};

typedef int (*RtmpMsgSend)(unsigned char* pBuff, int len);

typedef struct RtmpParam {
  RTMP_LogCallback* pFucRtmpLog;  //日志回调函数
  int LogLevel;                   //本地日志级别
  int AccIfPort;                  //接口机交互的端口
  int RtmpPort;                   // rtmp监听起始端口
  int WorkGroupid;                //进程号
  int RtmpTrunkSize;              // trunk大小
  int ListenOutIpFlag;            // rtmp使用外网还是内网
  int ConnectTimeOutMs;           //连接超时时间ms
  unsigned int
      ClientTimeOutMs;  //客户端超时时间，这个是客户端连接成功了但是超时没有数据过来
  int ExtTimeForFmt3;  //是否需要trunk3的扩展时间戳，谨慎设置，涉及到互通问题
  int EnableDumpFile;       //是否需要dump本地文件
  unsigned int UlsAppId;    //用于染色dump文件
  int FillZeroTailNumFlag;  //封装视频包是否每个帧头都带补0长度
  std::string Domain;
  int ResolveTimeOutMs;
  int ResolveIntervalMs;
  int DomainMod;
  int ConnectedMod;  // CONNECTED_MODE
  std::string TestSrvIP;

} RTMP_PARAM;

typedef struct TagRtmpProcessInfo {
  pid_t pid;
  int GroupId;
} RTMP_PROCESS_INFO;

typedef std::map<pid_t, RTMP_PROCESS_INFO> ProcInfoMap;

typedef struct TagRtmpConfig {
  int GroupNum;               //多少个处理进程
  int AccIfPort;              //接口机进程端口号
  int BasePort;               //进程监听起始端口号
  ProcInfoMap m_WorkProcMap;  //监控进程使用
  unsigned int UlsAppId;
  char LogFilePath[255];
  char ConfigFilePath[255];
  int LogLevel;                   //本地日志级别
  int EnableDumpFile;             //是否dump文件
  unsigned int ConnectTimeOutMs;  //连接超时时间，这个是连接过程的超时
  //客户端超时时间，这个是客户端连接成功了但是超时没有数据过来
  unsigned int ClientTimeOutMs;
  int ExtTimeForFmt3;          // trunk3是否带扩展时间戳
  int ListenOutIp;             // RTMP监听内网还是外网IP
  unsigned int RtmpTrunkSize;  // trunk大小
  int CpuNum;
  int MyCpuId;
  int FillZeroTailNumFlag;
  std::string Domain;
  int ResolveTimeOutMs;
  int ResolveIntervalMs;
  int ConnectedMod;

} RTMP_CONFIG;

typedef struct TagRtmpGlobalInfo {
  unsigned int RemoteIp = 0;
  unsigned long long SessionId = 0;
  unsigned long long TinyId = 0;
  unsigned int GroupId = 0;
} RTMP_GLOBAL_INFO;

typedef struct TagDomainAddr {
  struct sockaddr_in DominAddr;
  unsigned long long AliveTime;
} DOMAIN_ADDR_INFO;

extern RTMP_CONFIG g_RtmpCfg;

extern RTMP_GLOBAL_INFO g_RtmpGlbobalInfo;
class PeerConnection;

class RtmpPuller {
 public:
  explicit RtmpPuller(PeerConnection& pc);

  // note assignment and copy constructor
  ~RtmpPuller();

  int InitProtocolHandler(const std::string& Url);

  int HandlePacket();

  int CloseStream();

  // const ?
  int DownlinkPackAndSend(bool bAudio, const std::vector<char>& rawStream,
                          uint32_t rtpTime);

 private:
  std::vector<std::vector<char>> EncodeFec_(
      const std::vector<RtpBizPacket>& rtpBizPackets);
  // int GetRtmpSocket() const {return rtmp_.m_sb.sb_socket;}

  int WriteFile(unsigned char* pBuffer, unsigned int Len, FILE* fpFlv);

  void WriteFileAAC(FILE* pfOutfpAAC, ADTS_HEAD* pAdtsHead,
                    const RTMPPacket* pPacket);

  void WriteFileH264(FILE* pfOutfpH264, const RTMPPacket* pPacket);

  void WriteFileFlv(FILE* pfOutfpFlv, const RTMPPacket* pPacket);

  int WriteStream(char** buf, unsigned int* plen, unsigned int* nTimeStamp,
                  const RTMPPacket* pPacket);

  void Ontimer(Client* ClientList, int& Nfds);

  void Ontimer60s();

  unsigned int FlvProfile2AacProfile(const FlvAacProfile aacProfile);

  unsigned char* MakeClientMsg(char* pData, int DataLen, int& MsgLen,
                               Client* pClient);

  int SetupListen(int port);

  void SetupRtmp(int fd, int idx);

  int SendCreateStreamRes(RTMP* pRtmp, double Txn, double ID);

  int SendOnstatus(RTMP* pRtmp, double Txn, int streamid, int chan,
                   const char* level, const char* pCode, const char* pDesc);

  int SendPlayReset(RTMP* pRtmp, const char* pDesc);

  int SendPlayStart(RTMP* pRtmp, const char* pDesc);

  int SendPublishStart(RTMP* pRtmp, AMFObject* pObj, RTMPPacket* pPkg);

  int SendChunksize(RTMP* pRtmp, int Size);

  int SendErr(RTMP* pRtmp, double Txn, const char* pDesc);

  int SendConnectRes(RTMP* pRtmp, double Txn);

  int SendonFCPublish(RTMP* pRtmp);

  int PackVideoData(unsigned char* pDataBuf, int dataBufLen, unsigned int Ts,
                    int FrameType, Client* pClient);

  int TransVideo(Client* pClient, unsigned char* pBuff, int BufferLen);

  int TransAudio(Client* pClient, unsigned char* pBuff, int BufferLen);

  int HandleAccIfMsg(Client* pClient, unsigned char* pBuff, int BufferLen);

  int ProcessPlay(RTMP* pRtmp, AMFObject* pObj, RTMPPacket* pPkg);

  int ProcessPublish(RTMP* pRtmp, AMFObject* pObj, RTMPPacket* pPkg);

  int ProcessConnect(RTMP* pRtmp, AMFObject* pObj);

  int HandleChunksize(RTMP* pRtmp, RTMPPacket* pPacket);

  int HandleNotify(RTMP* pRtmp, RTMPPacket* pPkg);

  int HandleMyInvoke(Client* pClient, RTMPPacket* pPkg);

  int HandleVideoData(Client* pClient, const RTMPPacket* pPacket);

  int HandleAudioData(Client* pClient, const RTMPPacket* pPacket);

  int HandleControl(RTMP* pRtmp, RTMPPacket* pPkg);

  int HandleRtmpPacket(Client* pClient, RTMPPacket* pPkg);

  int HandleVideoSpsAndPps(Client* pClient, const RTMPPacket* pPacket);

  int HandleVideoSlice(Client* pClient, const RTMPPacket* pPacket);

  int GetFrameTypeAndPrepareSpsPps(Client* pClient, unsigned char* pNalu,
                                   char* pRawBuff, unsigned int& SendLen);

  int HandleVideoSliceStartCodeMod(Client* pClient, const RTMPPacket* pPacket);

  int HandleVideoSliceNormal(Client* pClient, const RTMPPacket* pPacket);

  void RecvUdpPkg(Client* ClientList, fd_set ReadSet, int& Nfds);

  void DoAcceptClient(Client* ClientList, fd_set ReadSet, int& Nfds);

  void DoRecvRtmpPkg(Client* ClientList, fd_set ReadSet, int& Nfds);

  void DoSendRtmpPkg(Client* ClientList, fd_set WriteSet, int& Nfds);

  void HandleTimeOutConnect(Client* ClientList, int& Nfds);

  int SendonResult(RTMP* pRtmp, double Txn);

  void HandleUdpPkg(Client* ClientList, int& Nfds, unsigned char* pBuff,
                    int PktLen);

  int RTMP_FindPrefixProperty(AMFObject* pObj, const AVal* pName,
                              AMFObjectProperty* pProperty);

  int AudioChannel(int stream_id);

  int VideoChannel(int stream_id);

  int DataChannel(int stream_id);

  int CalcChannel(int stream_id, int type_offset);

  void FeedDog(unsigned int StmNum, unsigned int FreeSolt);

  void GetFreeSoltNum();

  void StateTimeCost(int TimeCost, int Type);

  // int InitProtocolHandler(int Idx, int IsClientPlay);

  int HandleClientBW(RTMP* pRtmp, const RTMPPacket* pPacket);

  int SendMateData(Client* pClient);

  int HandleonStatus(Client* pClient, RTMPPacket* pPkg);

  int SendSpspAndPps(Client* pClient, unsigned char* pBuffer, unsigned int Len,
                     unsigned int timeStamp);

  int HandleHandlePbMsg(Client* ClientList, unsigned char* pBuff, int PktLen);

  void SendStmReport(Client* pClient, unsigned int TimePass);

  void SendStmDisConnect(Client* pClient, int Reason);

  int HandleReplaceUrl(std::string& Url, std::string& BakUrl);

  void InsertIp(struct sockaddr_in DomainAddr);

  int PackRtpAudio(Client* pClient, const unsigned char* pAudio, int Len,
                   int Ts);
  int HandleAACHead(Client* pClient, const RTMPPacket* pPkg);
  int HandleAudioHead(Client* pClient, const RTMPPacket* pPkg);
  int HandleAACRawData(Client* pClient, const RTMPPacket* pPkg);
  int HandleAudioRawData(Client* pClient, const RTMPPacket* pPkg);
  void DoTcpConnect(Client* ClientList, fd_set ReadSet, fd_set WriteSet,
                    int& Nfds);
  void HandleTimeOutRtmpHandshake(int i, int& Nfds);

 public:
  int m_AccUdpPort;
  int m_UdpPort;
  struct in_addr m_InnerAddr;
  int m_UdpFd;
  unsigned long long m_NowTimeMs;
  Client m_Clients[MAX_SLOT];
  RTMP rtmp_;  // librtmp ABI is disclosed
  PeerConnection& belongingPeerConnection_;
  int m_Socks[MAX_SLOT];  // tmp

 private:
  int m_RtmpPort;
  int m_StmId;
  struct in_addr m_OutAddr;
  int m_InitErr = 0;
  int m_StmNum;
  int m_PlayStmNum;
  int m_PlayActStmNum;
  int m_PublishStmNum;
  int m_PublishActStmNum;
  int m_FreeSolt;
  int m_TrunkSize = MAX_RTMP_TRUNK_SIZE;
  unsigned int m_ConnectTimeOutMs = 5000;
  unsigned int m_ClientTimeOutMs = 60000;
  int m_ExtTimeForFmt3 = 0;  // no use
  int m_ListenOutIp = 0;
  unsigned int m_UlsAppId;
  int m_GroupId;
  unsigned long long m_Pre1sTimeStamp;
  unsigned long long m_Pre5sTimeStamp;
  int m_FillZeroTailNumFlag;
  int m_WebrtcUdpClientPort;
  int m_ProcessType;
  std::string m_DomainName;
  DOMAIN_ADDR_INFO m_DomainIp[MAX_DOMAIN_NUM];
  struct in_addr m_BakAddr;
  int m_Idx;
  int m_ConnectedMod;  // CONNECTED_MODE
  int m_BframeStmNum;
};

void DumpFileStart(Client* pClient);

void DumpFileStop(Client* pClient);

extern int CreateDirectory(const std::string& Path);

extern in_addr_t GetAddrByName(const char* sIf);
extern const char* VideoDumpHex(const char* data, int len);

#endif  // RTMP_RTMP_PULL_H_
