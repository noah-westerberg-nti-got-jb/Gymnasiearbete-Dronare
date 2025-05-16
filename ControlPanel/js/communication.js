class SerialMessage {
	initiated;
	type;
	length;

	constructor() {
		this.initiated = false;
	}

	static messageNameFromType = new Map([
		[0, "controller-instructions"],
		[1, "pid-velocity"],
		[2, "pid-pitch"],
		[3, "pid-roll"],
		[4, "request-pid-velocity"],
		[5, "request-pid-pitch"],
		[6, "request-pid-roll"],
		[7, "target-ranges"],
		[8, "request-target-ranges"],
		[9, "acknowledge"],
		[10, "drone-log"],
		[11, "activate"],
		[12, "deactivate"],
		[13, "drone-velocity"],
		[14, "drone-acceleration"],
		[15, "drone-angular-velocity"],
		[16, "drone-angles"],
		[17, "drone-delta-time"],
		[18, "receiver-delta-time"],
		[19, "drone-motor-powers"],
		[20, "connection-status"]
	]);

	static messageTypeFromName = new Map([
		["controller-instructions", 0],
		["pid-velocity", 1],
		["pid-pitch", 2],
		["pid-roll", 3],
		["request-pid-velocity", 4],
		["request-pid-pitch", 5],
		["request-pid-roll", 6],
		["target-ranges", 7],
		["request-target-ranges", 8],
		["acknowledge", 9],
		["drone-log", 10],
		["activate", 11],
		["deactivate", 12],
		["drone-velocity", 13],
		["drone-acceleration", 14],
		["drone-angular-velocity", 15],
		["drone-angles", 16],
		["drone-delta-time", 17],
		["receiver-delta-time", 18],
		["drone-motor-powers", 19],
		["connection-status", 20]
	]);

	static messageLengths = new Map([
		[SerialMessage.messageTypeFromName.get("controller-instructions"), 3],
		[SerialMessage.messageTypeFromName.get("pid-velocity"), 12],
		[SerialMessage.messageTypeFromName.get("pid-pitch"), 12],
		[SerialMessage.messageTypeFromName.get("pid-roll"), 12],
		[SerialMessage.messageTypeFromName.get("request-pid-velocity"), 0],
		[SerialMessage.messageTypeFromName.get("request-pid-pitch"), 0],
		[SerialMessage.messageTypeFromName.get("request-pid-roll"), 0],
		[SerialMessage.messageTypeFromName.get("target-ranges"), 12],
		[SerialMessage.messageTypeFromName.get("request-target-ranges"), 0],
		[SerialMessage.messageTypeFromName.get("acknowledge"), 1],
		[SerialMessage.messageTypeFromName.get("drone-log"), 31],
		[SerialMessage.messageTypeFromName.get("activate"), 0],
		[SerialMessage.messageTypeFromName.get("deactivate"), 0],
		[SerialMessage.messageTypeFromName.get("drone-velocity"), 12],
		[SerialMessage.messageTypeFromName.get("drone-acceleration"), 12],
		[SerialMessage.messageTypeFromName.get("drone-angular-velocity"), 12],
		[SerialMessage.messageTypeFromName.get("drone-angles"), 12],
		[SerialMessage.messageTypeFromName.get("drone-delta-time"), 4],
		[SerialMessage.messageTypeFromName.get("receiver-delta-time"), 4],
		[SerialMessage.messageTypeFromName.get("drone-motor-powers"), 4],
		[SerialMessage.messageTypeFromName.get("connection-status"), 1]
	]);

	set(type) {
		if (SerialMessage.messageLengths.has(type)) {
			this.initiated = true;
			this.type = type;
			this.length = SerialMessage.messageLengths.get(type);
			return true;
		}
		return false;
	}

	reset() {
		this.initiated = false;
	}
}

class SerialReader {
	message;
	data;
	reading; 
	startMarker; 

	constructor() {
		this.message = new SerialMessage();
		this.data = [];
		this.reading = false;
		this.startMarker = 33;
	}

	read(byte) {
		if (this.reading) {
			if (!this.message.initiated) {
				if (!this.message.set(byte)) {
					this.reading = false;
				}
			} else if (this.data.length < this.message.length - 1) {
				this.data[this.data.length] = byte;
			} else {
				this.data[this.data.length] = byte;
				this.reading = false;
				const messageTypeName = SerialMessage.messageNameFromType.get(this.message.type);
				this.message.reset();
				const out = new Uint8Array(this.data);
				this.data = [];
				return {
					data: out,
					done: true,
					messageType: this.message.type,
					messageTypeName: messageTypeName,
					reading: true
				};
			}
		} else if (byte == this.startMarker) {
			this.reading = true;
		}

		return { done: false, reading: this.reading };
	}

	readPIDInstruction(data) {
		const floatValues = new Float32Array(data.buffer);
		return { p: floatValues[0], i: floatValues[1], d: floatValues[2] };
	}

	readTargetRanges(data) {
		const floatValues = new Float32Array(data.buffer);
		return { maxPitch: floatValues[0], maxRoll: floatValues[1], maxVerticalVelocity: floatValues[2] };
	}

	readVector(data) {
		const floatValues = new Float32Array(data.buffer);
		return { x: floatValues[0], y: floatValues[1], z: floatValues[2] };
	}

	readDeltaTime(data) {
		const floatValues = new Float32Array(data.buffer);
		return floatValues[0];
	}

	readMotorPowers(data) {
		const powers = new Int8Array(data.buffer);
		return {
			top_rigth: powers[0],
			top_left: powers[1],
			bottom_left: powers[2],
			bottom_right: powers[3]
		}
	}
}
const serialReader = new SerialReader();


class Terminal {
	decoder;
	currentMessage;
	currentIndex;
	previousLog;

	constructor() {
		this.decoder = new TextDecoder();
		this.currentMessage = new Uint8Array(64);
		this.currentIndex = 0;
		this.previousLog = "";
	}

	print() {
		const message = this.decoder.decode(this.currentMessage);
		this.addToTerminal(message);
		this.currentMessage = new Uint8Array(64);
		this.currentIndex = 0;
	}

	readByte(byte) {
		if (byte == 10 || byte == 13) {
			this.print();
			return;
		}
		else if (byte < 32 || byte > 126) {
			return;
		}

		if (this.currentIndex >= this.currentMessage.length) {
			this.print();
		}

		this.currentMessage[this.currentIndex++] = byte;
	}

	addToTerminal(log) {
		const output = document.getElementById("output");
		if (log == this.previousLog) {
			output.children[0].children[0].innerText = parseInt(output.children[0].children[0].innerText) + 1;
			return;
		}

		output.innerHTML = `<div><span>1</span>|${log}</div>` + output.innerHTML;
		this.previousLog = log;
		if (output.children.length > 100) {
			output.removeChild(output.lastChild);
		}
	}

	clear() {
		document.getElementById("output").innerHTML = "";
		this.previousLog = "";
	}
}
const terminal = new Terminal();


class WritingHandler {
	hasInstructions;
	currentInstructions;
	currentInstructionsType;
	confirmSendAcknowledge;

	constructor() {
		this.hasInstructions = false;
		this.currentInstructions = null;
		this.confirmSendAcknowledge = { hasConfirmed: true, value: null };
	}

	set(instructions, type) {
		this.currentInstructions = instructions;
		this.currentInstructionsType = type;
		this.hasInstructions = true;
	}

	setWithConfirm(instructions, type, value) {
		this.set(instructions, type);
		this.confirmSendAcknowledge = { hasConfirmed: false, value: value };
	}

	get() {
		if (this.hasInstructions) {
			return this.currentInstructions;
		}
		else {
			return sendControllerInstructions();
		}
	}

	acceptAcknowledge(messageType) {
		if (messageType != this.currentInstructionsType) {
			return;
		}

		if (!this.confirmSendAcknowledge.hasConfirmed) {
			const currentTypeName = SerialMessage.messageNameFromType.get(this.currentInstructionsType);
			const requestTypeName = `request-${currentTypeName}`;
			const requestType = SerialMessage.messageTypeFromName.get(requestTypeName);
			const requestInstructions = getRequestInstruction(requestType);
			this.set(requestInstructions, requestType);
			return;
		}

		this.hasInstructions = false;
		this.currentInstructions = null;
		this.currentInstructionsType = null;
		terminal.addToTerminal("[App] Acknowledge received for " + SerialMessage.messageNameFromType.get(this.currentInstructionsType));
	}

	confirmSendData(value, messageType) {
		if (this.confirmSendAcknowledge.hasConfirmed) {
			return;
		}

		if (this.confirmSendAcknowledge.value != value) {
			return;
		}

		this.confirmSendAcknowledge.hasConfirmed = true;
		this.acceptAcknowledge(messageType);
	}
}
const writingHandler = new WritingHandler();


const portSettings = { baudRate: 115200};
let port;
document.querySelector("#select-serial").addEventListener("click", async function() {
	port = await navigator.serial.requestPort();
	try {
		port.open(portSettings);
		setInterval(read);
		setInterval(write);
		
		terminal.addToTerminal("[App] Serial Port Connected");
	}
	catch (error) {
		terminal.addToTerminal("[App] " + error);
	}
});

let writing = false;
async function write() {
	if (writing || !port.writable) {
		return;
	}
	writing = true;

	const writer = port.writable.getWriter();

	const instructions = writingHandler.get();
	await writer.write(instructions);

	// Allow the serial port to be closed later.
	writer.releaseLock();
	writing = false;
}

let reading = false
async function read() {
	if (reading) {
		return;
	}
	reading = true;
	while (port.readable) {
		const reader = port.readable.getReader();
		try {
			while (true) {
				const { value, done } = await reader.read();
				if (done) {
					// Allow the serial port to be closed later.
					reader.releaseLock();
					console.log("exit read");
					break;
				}
				if (value) {
					for (const byte of value) {
						const result = serialReader.read(byte);

						if (!result.reading) {
							terminal.readByte(byte);
						}

						if (result.done) {
							// acknowledge
							if (result.messageTypeName == "acknowledge") {
								writingHandler.acceptAcknowledge(result.data[0]);
							}
							// pid
							else if (result.messageTypeName == "pid-velocity") {
								console.log(result);
								const pid = serialReader.readPIDInstruction(result.data);
								document.getElementById("pid-v-p").value = pid.p;
								document.getElementById("pid-v-i").value = pid.i;
								document.getElementById("pid-v-d").value = pid.d;
							}
							else if (result.messageTypeName == "pid-pitch") {
								const pid = serialReader.readPIDInstruction(result.data);
								document.getElementById("pid-p-p").value = pid.p;
								document.getElementById("pid-p-i").value = pid.i;
								document.getElementById("pid-p-d").value = pid.d;
							}
							else if (result.messageTypeName == "pid-roll") {
								const pid = serialReader.readPIDInstruction(result.data);
								document.getElementById("pid-r-p").value = pid.p;
								document.getElementById("pid-r-i").value = pid.i;
								document.getElementById("pid-r-d").value = pid.d;
							}

							// target values
							else if (result.messageTypeName == "target-ranges") {
								const ranges = serialReader.readTargetRanges(result.data);
								document.getElementById("terget-ranges-velocity").value = ranges.maxVerticalVelocity;
								document.getElementById("terget-ranges-pitch").value = ranges.maxPitch;
								document.getElementById("terget-ranges-roll").value = ranges.maxRoll;
							}

							// drone info
							else if (result.messageTypeName == "drone-velocity") {
								const velocity = serialReader.readVector(result.data);
								document.getElementById("vel-x").innerHTML = velocity.x.toFixed(2);
								document.getElementById("vel-y").innerHTML = velocity.y.toFixed(2);
								document.getElementById("vel-z").innerHTML = velocity.z.toFixed(2);
							}
							else if (result.messageTypeName == "drone-acceleration") {
								const acceleration = serialReader.readVector(result.data);
								document.getElementById("acc-x").innerHTML = acceleration.x.toFixed(2);
								document.getElementById("acc-y").innerHTML = acceleration.y.toFixed(2);
								document.getElementById("acc-z").innerHTML = acceleration.z.toFixed(2);
							}
							else if (result.messageTypeName == "drone-angular-velocity") {
								const angular_velocity = serialReader.readVector(result.data);
								document.getElementById("angular-vel-x").innerHTML = angular_velocity.x.toFixed(2);
								document.getElementById("angular-vel-y").innerHTML = angular_velocity.y.toFixed(2);
								document.getElementById("angular-vel-z").innerHTML = angular_velocity.z.toFixed(2);
							}
							else if (result.messageTypeName == "drone-angles") {
								const angles = serialReader.readVector(result.data);
								console.log("angles", angles);
								document.getElementById("pitch-value").innerHTML = angles.x.toFixed(2);
								document.getElementById("roll-value").innerHTML = angles.y.toFixed(2);
								document.getElementById("yaw-value").innerHTML = angles.z.toFixed(2);
							}

							// update rates
							else if (result.messageTypeName == "drone-delta-time") {
								const updateRate = 1 / serialReader.readDeltaTime(result.data);
								document.getElementById("drone-update").innerHTML = updateRate.toFixed(0);
							}
							else if (result.messageTypeName == "receiver-delta-time") {
								const updateRate = 1 / serialReader.readDeltaTime(result.data);
								document.getElementById("receiver-update").innerHTML = updateRate.toFixed(0);
							}

							// motor powers
							else if (result.messageTypeName == "drone-motor-powers") {
								const motorPowers = serialReader.readMotorPowers(result.data);
								document.getElementById("motor-fl").innerHTML = ((motorPowers.top_left + 127) / 255 * 100).toFixed(0) + "%";
								document.getElementById("motor-fr").innerHTML = ((motorPowers.top_rigth + 127) / 255 * 100).toFixed(0) + "%";
								document.getElementById("motor-bl").innerHTML = ((motorPowers.bottom_left + 127) / 255 * 100).toFixed(0) + "%";
								document.getElementById("motor-br").innerHTML = ((motorPowers.bottom_right + 127) / 255 * 100).toFixed(0) + "%";
							}

							// status indicators
							else if (result.messageTypeName == "connection-status") { 
								const status = result.data[0];
								const resetTime = 500;

								console.log("connection status", status, {receiver: (status & 1 << 0) != 0, drone: (status & 1 << 1) != 0, activated: (status & 1 << 2) != 0});

								setReceiverStatus("connected");
								clearTimeout(status_reset_timers.receiver);
								status_reset_timers.receiver = setTimeout(() => {setReceiverStatus("disconnected")}, resetTime);
								
								if ((status & (1 << 1)) == 0) { 
									continue;
								}

								setDroneConnectionStatus("connected");
								clearTimeout(status_reset_timers.drone);
								status_reset_timers.drone = setTimeout(() => {setDroneConnectionStatus("disconnected")}, resetTime);

								if ((status & (1 << 2)) == 0) { 
									continue;
								}
								setDroneActivivationStatus("connected");
								clearTimeout(status_reset_timers.activated);
								status_reset_timers.activated = setTimeout(() => {setDroneActivivationStatus("disconnected")}, resetTime);
							}
						}
					}
				}
			}
		} catch (error) {
			console.log(error);
		}
	}
	reading = false;
}

document.getElementById("open").addEventListener("click", async function() {
	if (port) {
		try {
			await port.close();
		} catch (error) {
			console.log(error);
		}
	}

	await port.open(portSettings);
});

function toInstruction(values, messageType) {
	const out = new Uint8Array(values.length + 2);
	out[0] = 33;
	out[1] = messageType;
	for (let i = 0; i < values.length; i++) {
		out[i + 2] = values[i];
	}
	return out;
}

// -1 <= x, y, power <= 1
function getControllerInstructions(x, y, power) {
	const values = new Int8Array(3);
	values[0] = (x * 127) & 0x0000ff;
	values[1] = (y * 127) & 0x0000ff;
	values[2] = (power * 127) & 0x0000ff;

	const messageType = SerialMessage.messageTypeFromName.get("controller-instructions");
	const uint8 = new Uint8Array(SerialMessage.messageLengths.get(messageType));
	uint8[0] = values[0];
	uint8[1] = values[1];
	uint8[2] = values[2];

	const out = toInstruction(uint8, messageType);

	return out;
}

function getPIDInstructions(values, module) {
	const floatValues = new Float32Array(3);
	floatValues[0] = values.p;
	floatValues[1] = values.i;
	floatValues[2] = values.d;

	const byteValues = new Uint8Array(floatValues.buffer);

	const messageType = SerialMessage.messageTypeFromName.get("pid-" + module);
	const out = toInstruction(byteValues, messageType);

	return out;
}

function getTargetRangeInstructions(maxPitch, maxRoll, maxVerticalVelocity) {
	const floatValues = new Float32Array(3);
	floatValues[0] = maxPitch;
	floatValues[1] = maxRoll;
	floatValues[2] = maxVerticalVelocity;

	const byteValues = new Uint8Array(floatValues.buffer);

	const messageType = SerialMessage.messageTypeFromName.get("target-ranges");

	const out = toInstruction(byteValues, messageType);

	return out;
}

function getRequestInstruction(type) {
	const out = toInstruction(new Uint8Array(0), type);
	return out;
}

document
	.getElementById("velocity")
	.querySelector(".send")
	.addEventListener("click", function() {
		const values = {
			p: document.getElementById("pid-v-p").value,
			i: document.getElementById("pid-v-i").value,
			d: document.getElementById("pid-v-d").value,
		};
		const instructions = getPIDInstructions(values, "velocity");

		writingHandler.setWithConfirm(instructions, SerialMessage.messageTypeFromName.get("pid-velocity"), values);
	});

document
	.getElementById("pitch")
	.querySelector(".send")
	.addEventListener("click", function() {
		const values = {
			p: document.getElementById("pid-p-p").value,
			i: document.getElementById("pid-p-i").value,
			d: document.getElementById("pid-p-d").value,
		};
		const instructions = getPIDInstructions(
			values,
			"pitch"
		);

		writingHandler.setWithConfirm(instructions, SerialMessage.messageTypeFromName.get("pid-pitch"), values);
	});

document
	.getElementById("roll")
	.querySelector(".send")
	.addEventListener("click", function() {
		const values = {
			p: document.getElementById("pid-r-p").value,
			i: document.getElementById("pid-r-i").value,
			d: document.getElementById("pid-r-d").value,
		};
		const instructions = getPIDInstructions(values, "roll");

		writingHandler.setWithConfirm(instructions, SerialMessage.messageTypeFromName.get("pid-roll"), values);
	});

document
	.getElementById("target-ranges")
	.querySelector(".send")
	.addEventListener("click", function() {
		const values = {
			maxPitch: document.getElementById("target-ranges-pitch").value,
			maxRoll: document.getElementById("target-ranges-roll").value,
			maxVerticalVelocity: document.getElementById("target-ranges-velocity").value,
		};
		const instructions = getTargetRangeInstructions(values.maxPitch, values.maxRoll, values.maxVerticalVelocity);

		writingHandler.setWithConfirm(instructions, SerialMessage.messageTypeFromName.get("target-ranges"), values);
	});

document
	.getElementById("velocity")
	.querySelector(".get")
	.addEventListener("click", function() {
		const instructions = getRequestInstruction(SerialMessage.messageTypeFromName.get("request-pid-velocity"));
		writingHandler.set(instructions, SerialMessage.messageTypeFromName.get("request-pid-velocity"));
	});

document
	.getElementById("pitch")
	.querySelector(".get")
	.addEventListener("click", function() {
		const instructions = getRequestInstruction(SerialMessage.messageTypeFromName.get("request-pid-pitch"));
		writingHandler.set(instructions, SerialMessage.messageTypeFromName.get("request-pid-pitch"));
	});

document
	.getElementById("roll")
	.querySelector(".get")
	.addEventListener("click", function() {
		const instructions = getRequestInstruction(SerialMessage.messageTypeFromName.get("request-pid-roll"));
		writingHandler.set(instructions, SerialMessage.messageTypeFromName.get("request-pid-roll"));
	});

document
	.getElementById("target-ranges")
	.querySelector(".get")
	.addEventListener("click", function() {
		const instructions = getRequestInstruction(SerialMessage.messageTypeFromName.get("request-target-ranges"));
		writingHandler.set(instructions, SerialMessage.messageTypeFromName.get("request-target-ranges"));
	});

document
	.getElementById("on")
	.addEventListener("click", function() {
		const instructions = getRequestInstruction(SerialMessage.messageTypeFromName.get("activate"));
		writingHandler.set(instructions, SerialMessage.messageTypeFromName.get("activate"));
	});

document
	.getElementById("off")
	.addEventListener("click", function() {
		const instructions = getRequestInstruction(SerialMessage.messageTypeFromName.get("deactivate"));
		writingHandler.set(instructions, SerialMessage.messageTypeFromName.get("deactivate"));
	});

document
	.getElementById("get-all")
	.addEventListener("click", async function() {
		const messageTypes = [
			SerialMessage.messageTypeFromName.get("request-pid-velocity"),
			SerialMessage.messageTypeFromName.get("request-pid-pitch"),
			SerialMessage.messageTypeFromName.get("request-pid-roll"),
			SerialMessage.messageTypeFromName.get("request-target-ranges")
		];

		for (const messageType of messageTypes) {
			const instructions = getRequestInstruction(messageType);
			writingHandler.set(instructions, messageType);
			await new Promise(resolve => {
                const checkInstructions = setInterval(() => {
                    if (!writingHandler.hasInstructions) {
                        clearInterval(checkInstructions);
                        resolve();
                    }
                }, 10);
            });
		};
	});

document
	.getElementById("send-all")
	.addEventListener("click", async function() {
		const pidValues = [
			{
				p: document.getElementById("pid-v-p").value,
				i: document.getElementById("pid-v-i").value,
				d: document.getElementById("pid-v-d").value,
				type: "velocity"
			},
			{
				p: document.getElementById("pid-p-p").value,
				i: document.getElementById("pid-p-i").value,
				d: document.getElementById("pid-p-d").value,
				type: "pitch"
			},
			{
				p: document.getElementById("pid-r-p").value,
				i: document.getElementById("pid-r-i").value,
				d: document.getElementById("pid-r-d").value,
				type: "roll"
			}
		];

		for (const values of pidValues) {
			const instructions = getPIDInstructions(values, values.type);
			writingHandler.setWithConfirm(instructions, SerialMessage.messageTypeFromName.get(`pid-${values.type}`), values);
			await new Promise(resolve => {
                const checkInstructions = setInterval(() => {
                    if (!writingHandler.hasInstructions) {
                        clearInterval(checkInstructions);
                        resolve();
                    }
                }, 10);
            });
		}

		const targetRanges = {
			maxPitch: document.getElementById("target-ranges-pitch").value,
			maxRoll: document.getElementById("target-ranges-roll").value,
			maxVerticalVelocity: document.getElementById("target-ranges-velocity").value
		};
		const instructions = getTargetRangeInstructions(targetRanges.maxPitch, targetRanges.maxRoll, targetRanges.maxVerticalVelocity);
		writingHandler.setWithConfirm(instructions, SerialMessage.messageTypeFromName.get("target-ranges"), targetRanges);
		await new Promise(resolve => {
			const checkInstructions = setInterval(() => {
				if (!writingHandler.hasInstructions) {
					clearInterval(checkInstructions);
					resolve();
				}
			}, 10);
		});
	});

function sendControllerInstructions() {
	const instructions = getControllerInstructions(
		-joystick.x,
		-joystick.y,
		document.getElementById("power").value
	);
	return instructions;
}


document.getElementById("clear").addEventListener("click", terminal.clear);

const status_colors = {
	connected: "green",
	disconnected: "red",
	unknown: "grey",
};

function setDroneActivivationStatus(status) {
	let color = status_colors.connected;
	if (status == "disconnected") {
		color = status_colors.disconnected;
	}
	else if (status == "unkown") {
		color = status_colors.unknown;
	}
	
	document.getElementById("drone-activation").style.color = color;
}

function setDroneConnectionStatus(status) {
	let color = status_colors.connected;
	if (status != "connected") {
		setDroneActivivationStatus("unkown");

		if (status == "disconnected") {
			color = status_colors.disconnected;
		}
		else if (status == "unkown") {
			color = status_colors.unknown;
		}
	}
	else {
		setDroneActivivationStatus("disconnected");
	}

	document.getElementById("drone-connection").style.color = color;
}

function setReceiverStatus(status) {
	let color = status_colors.connected;
	if (status != "connected") {
		setDroneConnectionStatus("unkown");
		
		if (status == "disconnected") {
			color = status_colors.disconnected;
		}
		else if (status == "unkown") {
			color = status_colors.unknown;
		}
	}

	document.getElementById("receiver-connection").style.color = color;
}

const status_reset_timers = {
	receiver: null,
	drone: null,
	activated: null,
};

setReceiverStatus("disconnected");