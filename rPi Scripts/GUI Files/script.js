var dataField = document.querySelector('.data'),
disabledBtn = document.querySelector('.disableBtn'),
standbyBtn = document.querySelector('.standbyBtn'),
runningBtn = document.querySelector('.runningBtn')

//Connect to RPi here
ip = "ws://127.0.0.1:8010/";
start(ip);

dataUpdate = 20;
refreshData() //This function is called recursively

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
    //Attempt connection
    websocket = new WebSocket(websocketServerLocation);

    //Right now, we assume the only message Prudentia sends is it's sharedData
    //This assumption can be changed later on if necessary
    websocket.onmessage = function (event) {
        data = JSON.parse(event.data);
        console.log(data["angularPosition"])
        dataField.textContent = "Data:" + JSON.stringify(data);
    };

    //If an error occurs, close socket. This will call websocket.onclose
    websocket.onerror = function(error){
        console.error("WebSocket error: Closing socket.");
        websocket.close();
    }

    //Retry connection after 2 seconds when socket closes
    websocket.onclose = function(event){
        retryTime = 2000;
        console.log("Websocket closed: ", event.reason, "Reconnecting in ", retryTime, " ms.")
        setTimeout(function(){start(websocketServerLocation)}, retryTime);
    }
}


function refreshData(){
    if (websocket.readyState === websocket.OPEN){
        console.log("Refreshing data")
        //Server returns data when the "messageType" field is "getData"
        var msg = {"messageType":"getData"}
        websocket.send(JSON.stringify(msg));
    }
    //Repeat this function again later
    setTimeout(refreshData, dataUpdate);
}


