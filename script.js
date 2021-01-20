var dataField = document.querySelector('.data'),
disabledBtn = document.querySelector('.disableBtn'),
standbyBtn = document.querySelector('.standbyBtn'),
runningBtn = document.querySelector('.runningBtn')

//Connect to RPi here
ip = "ws://127.0.0.1:6789/";
restarting = false
start(ip)
refreshData()

// Button callbacks
disabledBtn.onclick = function (event) {
    setState("shutdown")
}

standbyBtn.onclick = function (event) {
    setState("standby")
}

runningBtn.onclick = function (event) {
    setState("running")
}

function setState(state){
    var msg = {"messageType":"setState", "state":state}
    websocket.send(JSON.stringify(msg));
}

function start(websocketServerLocation){
    restarting = false;
    websocket = new WebSocket(websocketServerLocation);

    //Right now, we assume the only message Prudentia sends is it's sharedData
    websocket.onmessage = function (event) {
        data = JSON.parse(event.data);
        dataField.textContent = "Data:" + JSON.stringify(data);
    };

    websocket.onerror = function(error){
        console.error("WebSocket error: Closing socket.");
        websocket.close();
    }

    websocket.onclose = function(event){
        retryTime = 300
        console.log("Websocket closed: ", event.reason, "Reconnecting in ", retryTime, " ms.")
        setTimeout(function(){start(websocketServerLocation)}, retryTime);
        restarting = true;
    }
}


function refreshData(){
    if (websocket.readyState === websocket.OPEN){//Websocket open
        console.log("Refreshing data")
        var msg = {"messageType":"getData"}
        websocket.send(JSON.stringify(msg));
    }
    setTimeout(refreshData, 1000);
}


