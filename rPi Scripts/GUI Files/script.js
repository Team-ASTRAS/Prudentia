var dataMode = document.querySelector('.dataMode'),
    dataYaw = document.querySelector('.dataYaw'),
    dataPitch = document.querySelector('.dataPitch'),
    dataRoll = document.querySelector('.dataRoll'),
    dataSpeed = document.querySelector('.dataSpeed'),
    // SHUTDOWN, STABILIZE, STOP BUTTONS
    shutdown = document.querySelector('.Shutdown'),
    stabilize = document.querySelector('.Stabilize'),
    stop = document.querySelector('.StopButton'),
    // NAVIGATION BUTTONS
    homeNav = document.querySelector('.HomeButton'),
    instructionsNav = document.querySelector('.InstructionButton'),
    settingsNav = document.querySelector('.SettingsButton'),
    rtcNav = document.querySelector('.RTCButton'),
    aiNav = document.querySelector('.AIButton'),
    searchNav = document.querySelector('.SMButton'),
    // LOGGING BUTTONS
    startlog = document.querySelector('.StartLog'),
    stoplog = document.querySelector('.StopLog'),
    clearlog = document.querySelector('.ClearLog'),
    downloadlog = document.querySelector('.Save'),
    // RTC BUTTONS
    yawl = document.querySelector('.YawL'),
    yawr = document.querySelector('.YawR'),
    pitchu = document.querySelector('.PitchU'),
    pitchd = document.querySelector('.PitchD'),
    rollcw = document.querySelector('.RollCW'),
    rollccw = document.querySelector('.RollCCW'),
    // AI BUTTONS
    go = document.querySelector('.GO'),
    // SM BUTTONS
    searchmode = document.querySelector('.SearchButton'),

//Connect to RPi here
ip = "ws://127.0.0.1:8010/";
start(ip);

dataUpdate =  1000;
refreshData() //This function is called recursively

// Button callbacks
// SHUTDOWN, STABILIZE, STOP BUTTONS
shutdown.onclick = function (event) {
    console.log('Shutdown');
}
stabilize.onclick = function (event) {
    console.log('Stabilize');
}
stop.onclick = function (event) {
    console.log('Stop');
}
// NAVIGATION BUTTONS
homeNav.onclick = function (event) {
    console.log('Home Navigation');
    window.location = "Homepage.html";
}
instructionsNav.onclick = function (event) {
    console.log('Instructions Navigation');
    window.location = "InstructionPage.html";
}
settingsNav.onclick = function (event) {
    console.log('Settings Navigation');
    window.location = "SettingsPage.html";
}
rtcNav.onclick = function (event) {
    console.log('RTC Navigation');
    window.location = "RTCPage.html";
}
aiNav.onclick = function (event) {
    console.log('AI Navigation');
    window.location = "AIPage.html";
}
searchNav.onclick = function (event) {
    console.log('Search Navigation');
    window.location = "SMPage.html";
}
// LOGGING BUTTONS
if (startlog){
    startlog.onclick = function (event) {
        console.log('Start Logging');
      }
      stoplog.onclick = function (event) {
        console.log('Stop Logging');
      }
      clearlog.onclick = function (event) {
        console.log('Clear Log');
      }
      downloadlog.onclick = function (event) {
        console.log('Download Log');
      }
  }

  // RTC BUTTONS
  if (yawl){
  yawl.onmousedown = function (event) {
      yawl.style.background='red';
      if(document.getElementById('Speed5').checked){
        console.log('Yaw Left @ 5 deg/sec');
      }
      else if(document.getElementById('Speed10').checked){
        console.log('Yaw Left @ 10 deg/sec');
      }
      else if(document.getElementById('Speed15').checked){
        console.log('Yaw Left @ 15 deg/sec');
      }
      else if(document.getElementById('Speed30').checked){
        console.log('Yaw Left @ 30 deg/sec');
      }
      else if(document.getElementById('Speed50').checked){
        console.log('Yaw Left @ 50 deg/sec');
      }
      else{
        console.log('Yaw Left @ 20 deg/sec');
      }
  }
  yawl.onmouseup = function (event) {
      yawl.style.background='grey';
      console.log('Not Running')
  }

  yawr.onmousedown = function (event) {
      yawr.style.background='red';
      if(document.getElementById('Speed5').checked){
        console.log('Yaw Right @ 5 deg/sec');
      }
      else if(document.getElementById('Speed10').checked){
        console.log('Yaw Right @ 10 deg/sec');
      }
      else if(document.getElementById('Speed15').checked){
        console.log('Yaw Right @ 15 deg/sec');
      }
      else if(document.getElementById('Speed30').checked){
        console.log('Yaw Right @ 30 deg/sec');
      }
      else if(document.getElementById('Speed50').checked){
        console.log('Yaw Right @ 50 deg/sec');
      }
      else{
        console.log('Yaw Right @ 20 deg/sec');
      }
  }
  yawr.onmouseup = function (event) {
      yawr.style.background='grey';
      console.log('Not Running')
  }

  pitchu.onmousedown = function (event) {
      pitchu.style.background='blue';
      if(document.getElementById('Speed5').checked){
        console.log('Pitch Up @ 5 deg/sec');
      }
      else if(document.getElementById('Speed10').checked){
        console.log('Pitch Up @ 10 deg/sec');
      }
      else if(document.getElementById('Speed15').checked){
        console.log('Pitch Up @ 15 deg/sec');
      }
      else if(document.getElementById('Speed30').checked){
        console.log('Pitch Up @ 30 deg/sec');
      }
      else if(document.getElementById('Speed50').checked){
        console.log('Pitch Up @ 50 deg/sec');
      }
      else{
        console.log('Pitch Up @ 20 deg/sec');
      }
  }
  pitchu.onmouseup = function (event) {
      pitchu.style.background='grey';
      console.log('Not Running')
  }

  pitchd.onmousedown = function (event) {
      pitchd.style.background='blue';
      if(document.getElementById('Speed5').checked){
        console.log('Pitch Down @ 5 deg/sec');
      }
      else if(document.getElementById('Speed10').checked){
        console.log('Pitch Down @ 10 deg/sec');
      }
      else if(document.getElementById('Speed15').checked){
        console.log('Pitch Down @ 15 deg/sec');
      }
      else if(document.getElementById('Speed30').checked){
        console.log('Pitch Down @ 30 deg/sec');
      }
      else if(document.getElementById('Speed50').checked){
        console.log('Pitch Down @ 50 deg/sec');
      }
      else{
        console.log('Pitch Down @ 20 deg/sec');
      }
  }
  pitchd.onmouseup = function (event) {
      pitchd.style.background='grey';
      console.log('Not Running')
  }

  rollcw.onmousedown = function (event) {
      rollcw.style.background='green';
      if(document.getElementById('Speed5').checked){
        console.log('Roll CW @ 5 deg/sec');
      }
      else if(document.getElementById('Speed10').checked){
        console.log('Roll CW @ 10 deg/sec');
      }
      else if(document.getElementById('Speed15').checked){
        console.log('Roll CW @ 15 deg/sec');
      }
      else if(document.getElementById('Speed30').checked){
        console.log('Roll CW @ 30 deg/sec');
      }
      else if(document.getElementById('Speed50').checked){
        console.log('Roll CW @ 50 deg/sec');
      }
      else{
        console.log('Roll CW @ 20 deg/sec');
      }
  }
  rollcw.onmouseup = function (event) {
      rollcw.style.background='grey';
      console.log('Not Running')
  }

  rollccw.onmousedown = function (event) {
      rollccw.style.background='green';
      if(document.getElementById('Speed5').checked){
        console.log('Roll CCW @ 5 deg/sec');
      }
      else if(document.getElementById('Speed10').checked){
        console.log('Roll CCW @ 10 deg/sec');
      }
      else if(document.getElementById('Speed15').checked){
        console.log('Roll CCW @ 15 deg/sec');
      }
      else if(document.getElementById('Speed30').checked){
        console.log('Roll CCW @ 30 deg/sec');
      }
      else if(document.getElementById('Speed50').checked){
        console.log('Roll CCW @ 50 deg/sec');
      }
      else{
        console.log('Roll CCW @ 20 deg/sec');
      }
  }
  rollccw.onmouseup = function (event) {
      rollccw.style.background='grey';
      console.log('Not Running')
  }
}

// AI BUTTONS
if (go){
  go.onclick = function (event) {
      yawTarget = document.getElementById("YawTarget").value;
      pitchTarget = document.getElementById("PitchTarget").value;
      rollTarget = document.getElementById("RollTarget").value;
      console.log('Target Yaw = ' + yawTarget);
      console.log('Target Pitch = ' + pitchTarget);
      console.log('Target Roll = ' + rollTarget);
  }
}

// SM BUTTONS
if (searchmode){
  searchmode.onclick = function (event) {
      console.log('Search Mode');
  }
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
        dataMode.textContent = data["state"]
        dataYaw.textContent = data["angularPosition"][0];
        dataPitch.textContent = data["angularPosition"][1];
        dataRoll.textContent = data["angularPosition"][2];
        dataSpeed.textContent = data["angularVelocityMagnitude"];
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
