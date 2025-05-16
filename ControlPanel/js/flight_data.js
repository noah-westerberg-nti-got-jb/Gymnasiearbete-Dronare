let flightData = [];
let startTime;
let lastDataPoint = {};
let collection;
let collecting = false;

function startCollectingFlightData() {
    startTime = new Date();
    flightData = [];
    collection = setInterval(collectFlightData, 10);
    collecting = true;
}

function stopCollectingFlightData() {
    if (!collecting) {
        return;
    }

    clearInterval(collection);
    const endTime = new Date();
    const dataToSave = {
        startDateTime: startTime.toISOString(),
        endDateTime: endTime.toISOString(),
        data: flightData
    };
    saveFlightDataToFile(dataToSave);

    collecting = false;
}

function saveFlightDataToFile(data) {
    const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `flight_data_${startTime.toISOString()}.json`;
    a.click();
    URL.revokeObjectURL(url);
}

function hasDataChanged(newDataPoint) {
    const { time: newTime, ...newData } = newDataPoint;
    const { time: lastTime, ...lastData } = lastDataPoint;
    return JSON.stringify(newData) !== JSON.stringify(lastData);
}

function collectFlightData() {
    const currentTime = new Date();
    const elapsedTime = (currentTime - startTime) / 1000;
    const dataPoint = {
        time: elapsedTime,
        acceleration: {
            x: parseFloat(document.getElementById("acc-x").innerText),
            y: parseFloat(document.getElementById("acc-y").innerText),
            z: parseFloat(document.getElementById("acc-z").innerText)
        },
        velocity: {
            x: parseFloat(document.getElementById("vel-x").innerText),
            y: parseFloat(document.getElementById("vel-y").innerText),
            z: parseFloat(document.getElementById("vel-z").innerText)
        },
        angularVelocity: {
            x: parseFloat(document.getElementById("angular-vel-x").innerText),
            y: parseFloat(document.getElementById("angular-vel-y").innerText),
            z: parseFloat(document.getElementById("angular-vel-z").innerText)
        },
        rotation: {
            pitch: parseFloat(document.getElementById("pitch-value").innerText),
            roll: parseFloat(document.getElementById("roll-value").innerText),
            yaw: parseFloat(document.getElementById("yaw-value").innerText)
        },
        controllerPower: parseFloat(document.getElementById("power-info").innerText),
        joystick: {
            x: parseFloat(document.getElementById("joystick-x-info").innerText),
            y: parseFloat(document.getElementById("joystick-y-info").innerText)
        },
        fps: {
            app: parseInt(document.getElementById("app-update").innerText),
            drone: parseInt(document.getElementById("drone-update").innerText),
            receiver: parseInt(document.getElementById("receiver-update").innerText)
        },
        motorPowers: {
            frontLeft: parseFloat(document.getElementById("motor-fl").innerText.replace('%', '')),
            frontRight: parseFloat(document.getElementById("motor-fr").innerText.replace('%', '')),
            backLeft: parseFloat(document.getElementById("motor-bl").innerText.replace('%', '')),
            backRight: parseFloat(document.getElementById("motor-br").innerText.replace('%', ''))
        }
    };

    if (hasDataChanged(dataPoint)) {
        flightData.push(dataPoint);
        lastDataPoint = dataPoint;
    }
}

document.getElementById("on").addEventListener("click", startCollectingFlightData);

document.getElementById("off").addEventListener("click", stopCollectingFlightData);