const kFrameRate: number = 20;
const kSideLenPix: number = 500;
const kStatBoxMsgNum: number = 20;
let pc: RTCPeerConnection | null = null;
let sendChannel: RTCDataChannel | null = null;
let sendButton: HTMLButtonElement | null = null;
let messageInputBox: HTMLInputElement | null = null;
let g_vp8Payload: string | null = null;

function addToStatBox(text: string, color: string): void {
    const messageElement: HTMLDivElement = document.createElement('div');
    messageElement.textContent = `${getTimeString()} ${text}`;
    messageElement.style.color = color;

    const statBox: HTMLElement | null = document.getElementById('statBoxID');
    if (statBox) {
        statBox.appendChild(messageElement);
        let messages: NodeListOf<HTMLDivElement> = statBox.querySelectorAll('div');
        while (messages.length > kStatBoxMsgNum) {
            statBox.removeChild(messages[0]);
            messages = statBox.querySelectorAll('div');
        }
    }
}

async function getStat(): Promise<any | null> {
    if (pc == null) {
        console.log("pc is nil");
        return null;
    }

    let activeCandidatePair: any;
    const stats: RTCStatsReport = await pc.getStats();
    stats.forEach((report: RTCStats) => {
        console.log(`stat report=${JSON.stringify(report)}`);
        if (report.type === 'transport') {
            activeCandidatePair = (<any>stats).get((<RTCTransportStats>report).selectedCandidatePairId);
        }
    });

    return activeCandidatePair;
}

async function printCandidate(): Promise<void> {
    const candidatePair = await getStat();
    if (candidatePair == null) {
        return;
    }

    const text: string = `RTT=${candidatePair.currentRoundTripTime * 1000} ms via STUN, s/r ${candidatePair.requestsSent}/${candidatePair.responsesReceived}.`;
    addToStatBox(text, "green");
}

setInterval(printCandidate, 2000);

async function getRemoteStream(e: RTCTrackEvent): Promise<void> {
    const videoElement: HTMLVideoElement | null = document.getElementById('video_container_subscribe') as HTMLVideoElement;
    if (videoElement) {
        videoElement.width = videoElement.height = kSideLenPix;
    }

    console.log(`ontrack recv event=${JSON.stringify(e)}`);
    if (!e.streams) {
        return;
    }

    const downStream: MediaStream = e.streams[0];
    if (videoElement) {
        videoElement.srcObject = downStream;
    }

    downStream.getVideoTracks()[0].onmute = (event: Event) => {
        console.log(`video recv onmute=${JSON.stringify(event)}`);
    };

    downStream.getVideoTracks()[0].onunmute = (event: Event) => {
        console.log(`video recv onunmute=${JSON.stringify(event)}`);
    };

    downStream.getAudioTracks()[0].onmute = (event: Event) => {
        console.log(`audio recv onmute=${JSON.stringify(event)}`);
    };

    downStream.getAudioTracks()[0].onunmute = (event: Event) => {
        console.log(`audio recv onunmute=${JSON.stringify(event)}`);
    };
}

function sendMessage(): void {
    if (sendChannel && messageInputBox) {
        sendChannel.send(messageInputBox.value);
        addToDialog('me: ' + messageInputBox.value);
        messageInputBox.value = "";
        messageInputBox.focus();
    }
}

function getTimeString(): string {
    const now: Date = new Date();
    const hours: string = now.getHours().toString().padStart(2, '0');
    const minutes: string = now.getMinutes().toString().padStart(2, '0');
    const seconds: string = now.getSeconds().toString().padStart(2, '0');

    return `${hours}:${minutes}:${seconds}`;
}

function addToDialog(strMessage: string): void {
    const text: string = `${getTimeString()} ${strMessage}`;
    const txtNode: Text = document.createTextNode(text);
    const el: HTMLParagraphElement = document.createElement("p");
    el.appendChild(txtNode);
    const receiveBox: HTMLElement | null = document.getElementById('receivebox');
    if (receiveBox) {
        receiveBox.appendChild(el);
    }
}

function handleReceiveMessage(event: MessageEvent): void {
    try {
        const o: any = JSON.parse(event.data);
        if (!(o && typeof o === "object")) {
            addToDialog('friend: ' + event.data);
            return;
        }

        if (o.s2cStateReq) {
            addToStatBox(`RTT=${o.s2cStateReq.uint32RttMs} ms via RTCP in server.`, "black");
        } else {
            addToDialog('recv unknown: ' + event.data);
        }
    } catch (e) {
        addToDialog('friend: ' + event.data);
    }
}

function handleSendChannelStatusChange(event: Event): void {
    if (sendChannel) {
        const state: string = sendChannel.readyState;
        if (state === "open") {
            console.log("data channel opened");
            addToDialog("connect success");
            if (messageInputBox) {
                messageInputBox.disabled = false;
            }
            if (sendButton) {
                sendButton.disabled = false;
            }
            sendChannel.send(`g_vp8Payload=${g_vp8Payload}`);
        } else {
            console.log("data channel closed");
            addToDialog("closed");
            if (messageInputBox) {
                messageInputBox.disabled = true;
            }
            if (sendButton) {
                sendButton.disabled = true;
            }
        }
    }
}

async function unpublish(): Promise<void> {
    console.log("To unpublish");
    const videoElement: HTMLVideoElement | null = document.getElementById('video_container_publish') as HTMLVideoElement;

    const tracks: MediaStreamTrack[] = (<MediaStream>videoElement?.srcObject)?.getTracks() || [];
    tracks.forEach((track: MediaStreamTrack) => {
        track.stop();
    });

    if (videoElement) {
        videoElement.srcObject = null;
    }

    if (pc) {
        pc.close();
        pc = null;
    }
}

async function publish(): Promise<void> {
    messageInputBox = document.getElementById('messageid') as HTMLInputElement;
    sendButton = document.getElementById('sendButtonid') as HTMLButtonElement;

    console.log("To publish");

    const videoElement: HTMLVideoElement | null = document.getElementById('video_container_publish') as HTMLVideoElement;
    if (videoElement) {
        videoElement.width = videoElement.height = kSideLenPix;
    }

    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
        console.error('the getUserMedia is not supported');
        return;
    }

    const constraints: MediaStreamConstraints = {
        video: {
            facingMode: { ideal: 'user' },
            frameRate: { max: kFrameRate, ideal: kFrameRate },
            width: { max: kSideLenPix, ideal: kSideLenPix },
            height: { max: kSideLenPix, ideal: kSideLenPix },
            aspectRatio: 1,
        },
        audio: {
            sampleRate: { ideal: 48000 },
            channelCount: { ideal: 1 },
            echoCancellation: { ideal: true },
            autoGainControl: { ideal: true },
            noiseSuppression: { ideal: true },
        }
    };

    console.log(`to get media, constraints=${JSON.stringify(constraints, null, 2)}`);
    const stream: MediaStream = await navigator.mediaDevices.getUserMedia(constraints);

    console.log("to get media done");
    if (videoElement) {
        videoElement.srcObject = stream;
    }

    pc = new RTCPeerConnection({
        iceServers: [],
        iceTransportPolicy: "all"
    });

    if (stream.getAudioTracks()[0]) {
        await pc.addTransceiver(stream.getAudioTracks()[0], {
            direction: "sendrecv",
            streams: [stream],
        });
    }
    if (stream.getVideoTracks()[0]) {
        await pc.addTransceiver(stream.getVideoTracks()[0], {
            direction: "sendrecv",
            streams: [stream],
        });
    }

    await pc.addTransceiver("audio", {
        direction: "recvonly",
    });

    await pc.addTransceiver("video", {
        direction: "recvonly",
    });

    pc.ontrack = getRemoteStream;
    const offer: RTCSessionDescriptionInit = await pc.createOffer();
    offer.sdp = offer.sdp.replaceAll('profile-level-id', 'sps-pps-idr-in-keyframe=1;profile-level-id');
    await pc.setLocalDescription(offer);

    console.log("=============== offer sdp:");
    console.log(offer.sdp);
    g_vp8Payload = offer.sdp.match("a=rtpmap:([0-9]+) VP8")[1];
    console.log(`offer vp8 payload=${g_vp8Payload}`);

    let answerSDP: string = `v=0
o=- 7595655801978680453 1 IN IP4 127.0.0.1
s=media
c=IN IP4 0.0.0.0
t=0 0
a=ice-lite
a=msid-semantic: WMS *
a=group:BUNDLE 0 1 2 3
m=audio 9 UDP/TLS/RTP/SAVPF 111
a=rtcp:9 IN IP4 0.0.0.0
a=candidate:1 1 udp 1686052607 36.155.205.204 8090 typ host
a=ice-ufrag:144115262675375852
a=ice-pwd:728c90257276ed9e48aac959
a=fingerprint:sha-256 55:9B:3E:43:91:2D:66:79:0F:B6:84:45:B3:47:3F:56:D4:4C:9E:10:EA:B4:66:C6:64:42:42:4B:5C:CD:FF:2F
a=setup:passive
a=mid:0
a=extmap:1 urn:ietf:params:rtp-hdrext:ssrc-audio-level
a=extmap:2 http://www.webrtc.org/experiments/rtp-hdrext/abs-send-time
a=recvonly
a=rtcp-mux
a=rtpmap:111 opus/48000/2
a=rtcp-fb:111 nack
a=rtcp-fb:111 rrtr
a=fmtp:111 minptime=20;sprop-stereo=0;stereo=0
m=video 9 UDP/TLS/RTP/SAVPF 96
c=IN IP4 0.0.0.0
a=rtcp:9 IN IP4 0.0.0.0
a=candidate:1 1 udp 1686052607 36.155.205.204 8090 typ host
a=ice-ufrag:144115262675375852
a=ice-pwd:728c90257276ed9e48aac959
a=fingerprint:sha-256 55:9B:3E:43:91:2D:66:79:0F:B6:84:45:B3:47:3F:56:D4:4C:9E:10:EA:B4:66:C6:64:42:42:4B:5C:CD:FF:2F
a=setup:passive
a=mid:1
a=extmap:2 http://www.webrtc.org/experiments/rtp-hdrext/abs-send-time
a=extmap:14 urn:ietf:params:rtp-hdrext:toffset
a=recvonly
a=rtcp-mux
a=rtcp-rsize
a=rtpmap:96 VP8/90000
a=rtcp-fb:96 goog-remb
a=rtcp-fb:96 ccm fir
a=rtcp-fb:96 nack
a=rtcp-fb:96 nack pli
a=rtcp-fb:96 rrtr
m=audio 9 UDP/TLS/RTP/SAVPF 111
c=IN IP4 0.0.0.0
a=rtcp:9 IN IP4 0.0.0.0
a=candidate:1 1 udp 1686052607 36.155.205.204 8090 typ host
a=ice-ufrag:144115262675375852
a=ice-pwd:728c90257276ed9e48aac959
a=fingerprint:sha-256 55:9B:3E:43:91:2D:66:79:0F:B6:84:45:B3:47:3F:56:D4:4C:9E:10:EA:B4:66:C6:64:42:42:4B:5C:CD:FF:2F
a=setup:passive
a=mid:2
a=extmap:1 urn:ietf:params:rtp-hdrext:ssrc-audio-level
a=extmap:2 http://www.webrtc.org/experiments/rtp-hdrext/abs-send-time
a=sendonly
a=rtcp-mux
a=rtpmap:111 opus/48000/2
a=rtcp-fb:111 nack
a=rtcp-fb:111 rrtr
a=fmtp:111 minptime=20;sprop-stereo=0;stereo=0
a=ssrc:16854838 cname:YZcxBwerFFm6GH69
a=ssrc:16854838 msid:5Y2wZK8nANNAoVw6dSAHVjNxrD1ObBM2kBPV 128f4fa0-81dd-4c3a-bbcd-22e71e29d178
a=ssrc:16854838 mslabel:5Y2wZK8nANNAoVw6dSAHVjNxrD1ObBM2kBPV
a=ssrc:16854838 label:128f4fa0-81dd-4c3a-bbcd-22e71e29d178
m=video 9 UDP/TLS/RTP/SAVPF 96 106
c=IN IP4 0.0.0.0
a=rtcp:9 IN IP4 0.0.0.0
a=candidate:1 1 udp 1686052607 36.155.205.204 8090 typ host
a=ice-ufrag:144115262675375852
a=ice-pwd:728c90257276ed9e48aac959
a=fingerprint:sha-256 55:9B:3E:43:91:2D:66:79:0F:B6:84:45:B3:47:3F:56:D4:4C:9E:10:EA:B4:66:C6:64:42:42:4B:5C:CD:FF:2F
a=setup:passive
a=mid:3
a=extmap:2 http://www.webrtc.org/experiments/rtp-hdrext/abs-send-time
a=extmap:14 urn:ietf:params:rtp-hdrext:toffset
a=sendonly
a=rtcp-mux
a=rtcp-rsize
a=rtpmap:96 VP8/90000
a=rtcp-fb:96 goog-remb
a=rtcp-fb:96 ccm fir
a=rtcp-fb:96 nack
a=rtcp-fb:96 nack pli
a=rtcp-fb:96 rrtr
a=rtpmap:106 H264/90000
a=fmtp:106 packetization-mode=1;profile-level-id=42e01f;sps-pps-idr-in-keyframe=1;level-asymmetry-allowed=1
a=rtcp-fb:106 goog-remb
a=rtcp-fb:106 ccm fir
a=rtcp-fb:106 nack
a=rtcp-fb:106 nack pli
a=rtcp-fb:106 rrtr
a=ssrc-group:FID 33697348 50472669
a=ssrc:33697348 cname:ovaCctnHP9Asci9c
a=ssrc:33697348 msid:5Y2wZK8nANNAoVw6dSAHVjNxrD1ObBM2kBPV 1d7fc300-9889-4f94-9f35-c0bcc77a260d
a=ssrc:33697348 mslabel:5Y2wZK8nANNAoVw6dSAHVjNxrD1ObBM2kBPV
a=ssrc:33697348 label:1d7fc300-9889-4f94-9f35-c0bcc77a260d
a=ssrc:50472669 cname:ovaCctnHP9Asci9c
a=ssrc:50472669 msid:5Y2wZK8nANNAoVw6dSAHVjNxrD1ObBM2kBPV 1d7fc300-9889-4f94-9f35-c0bcc77a260d
a=ssrc:50472669 mslabel:5Y2wZK8nANNAoVw6dSAHVjNxrD1ObBM2kBPV
a=ssrc:50472669 label:1d7fc300-9889-4f94-9f35-c0bcc77a260d
`;

    console.log("=============== answer sdp:");
    console.log(answerSDP);

    const answer: RTCSessionDescriptionInit = new RTCSessionDescription({
        type: 'answer',
        sdp: answerSDP
    });

    console.log("print answer done");
    await pc.setRemoteDescription(answer);
}

window.addEventListener('load', publish, false);