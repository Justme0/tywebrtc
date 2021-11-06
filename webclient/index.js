// form https://github.com/notedit/whip-server

const Express		= require("express");
const BodyParser	= require("body-parser");

const FS		= require("fs");
const Path		= require("path");

//Get the Medooze Media Server interface
const MediaServer = require("medooze-media-server");

//Get Semantic SDP objects
const SemanticSDP	= require("semantic-sdp");
const SDPInfo		= SemanticSDP.SDPInfo;
const MediaInfo		= SemanticSDP.MediaInfo;
const CandidateInfo	= SemanticSDP.CandidateInfo;
const DTLSInfo		= SemanticSDP.DTLSInfo;
const ICEInfo		= SemanticSDP.ICEInfo;
const StreamInfo	= SemanticSDP.StreamInfo;
const TrackInfo		= SemanticSDP.TrackInfo;
const Direction		= SemanticSDP.Direction;
const CodecInfo		= SemanticSDP.CodecInfo;

const Capabilities = {
	audio : {
		codecs		: ["opus"],
		extensions	: [ "urn:ietf:params:rtp-hdrext:ssrc-audio-level"]
	},
	video : {
		codecs		: ["h264"],
		rtx		: true,
		rtcpfbs		: [
			{ "id": "transport-cc"},
			{ "id": "ccm", "params": ["fir"]},
			{ "id": "nack"},
			{ "id": "nack", "params": ["pli"]}
		],
		extensions	: [ "http://www.ietf.org/id/draft-holmer-rmcat-transport-wide-cc-extensions-01"]
	}
};


MediaServer.enableDebug(false);
MediaServer.enableUltraDebug(false);

//Restrict port range
MediaServer.setPortRange(10000,50000);


//Create UDP server endpoint
const endpoint = MediaServer.createEndpoint('127.0.0.1');



//Create rest api
const rest = Express();
rest.use(function(req, res, next) {
	res.header("Access-Control-Allow-Origin", "*");
	res.header("Access-Control-Allow-Headers", "*");
	res.header("Access-Control-Allow-Methods", "POST, GET, DELETE, PUT, OPTIONS");
	next();
});
rest.use(BodyParser.text({type:"application/sdp"}));

rest.use(Express.static('static'))

rest.get("/", (req,res) => {
    res.sendFile('index.html', { root: __dirname + '/static'})
})


rest.use(BodyParser.text({type:"application/sdp"}));
rest.post("/whip/:streamId" , (req, res)=>{
	//Get body
    const body = req.body;
	//Get streamId
	const streamId = req.params.streamId;
    //Get offer
    const offer =  SDPInfo.parse(body);

	const transport = endpoint.createTransport(offer);
	
	transport.on("icetimeout" ,()=>{
		console.log(streamId +"::icetimeout");
		transport.stop();
	});
	transport.on("dtlsstate" ,(state)=>{
		console.log(streamId +"::dtlsstate::"+state);
		if (state=="failed" || state=="closed")
			transport.stop();
	});
		
			
	//Set RTP remote properties
	transport.setRemoteProperties(offer);
			
	//Create local SDP info
	const answer = offer.answer({
		dtls		: transport.getLocalDTLSInfo(),
		ice		: transport.getLocalICEInfo(),
		candidates	: endpoint.getLocalCandidates(),
		capabilities	: Capabilities
	});

	//Set RTP local  properties
	transport.setLocalProperties(answer);

	//Done
	res.type("application/sdp");
	res.send(answer.toString());
});


rest.listen(8080,"0.0.0.0", ()=> {
	console.log("Start listening port 8080");
})
