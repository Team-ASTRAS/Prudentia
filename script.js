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
    setState("disabled")
}

standbyBtn.onclick = function (event) {
    setState("standby")
}

runningBtn.onclick = function (event) {
    setState("running")
}

function start(websocketServerLocation){
    restarting = false;
    websocket = new WebSocket(websocketServerLocation);

    websocket.onmessage = function (event) {
        data = JSON.parse(event.data);
        dataField.textContent = "Data:" + JSON.stringify(data);
    };

    //Retry connection
    websocket.onclose = function(event){
        if(!restarting){ /* Avoid firing a new setInterval, after one has been done */
            setInterval(function(){start(websocketServerLocation)}, 3000);
            restarting = true;
        }
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

function setState(state){
    var msg = {"messageType":"setState", "state":state}
    websocket.send(JSON.stringify(msg));
}

