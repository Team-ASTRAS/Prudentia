var script1 = document.createElement('script');
script1.src = 'https://ajax.googleapis.com/ajax/libs/jquery/1.4.1/jquery.js';
script1.type = 'text/javascript';
document.getElementsByTagName('head')[0].appendChild(script1);
var script2 = document.createElement('script');
script2.src = 'https://cdnjs.cloudflare.com/ajax/libs/Chart.js/2.7.2/Chart.bundle.js';
script2.type = 'text/javascript';
document.getElementsByTagName('head')[0].appendChild(script2);

var dataMode = document.querySelector('.dataMode'),
    dataYaw = document.querySelector('.dataYaw'),
    dataPitch = document.querySelector('.dataPitch'),
    dataRoll = document.querySelector('.dataRoll'),
    dataSpeed = document.querySelector('.dataSpeed'),
    calibrated, // Variable should come from IMU instead of button press
    ready, // Variable should come from IMU/Motors instead of button press
    // SHUTDOWN, STABILIZE, STOP BUTTONS
    shutdown = document.querySelector('.Shutdown'),
    stabilize = document.querySelector('.Stabilize'),
    stop = document.querySelector('.StopButton'),
    calibrate = document.querySelector('.Calibrate'),
    spinup = document.querySelector('.Spinup'),
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
    var quit = confirm('Did you remember to save your data? Click CANCEL to go back and save your data. Click OK to continue the shut down process. ');
    if (quit == true){
      alert('Hold Prudentia steady while the motors spin down. Click OK to begin spin down.');
      console.log('Shutdown');
    }
}
stabilize.onclick = function (event) {
  if (ready == true){           // Change to if mode = running
    console.log('Stabilize');
  }
}
if (calibrate){
  calibrate.onclick = function (event) {
      alert('Align Prudentia with the calibration target. Click OK when Prudentia is aligned.');
      calibrated = true;
      console.log('System Calibrated');
  }
  spinup.onclick = function (event) {
      if (calibrated == true){
        alert('Hold Prudentia steady while the motors spin up. Click OK to begin spin up.');
        ready = true;           //mode = running
        console.log('Spinup');
    }
  }
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
  if (ready == true){               // mode = running
      console.log('RTC Navigation');
      window.location = "RTCPage.html";
  }
}
aiNav.onclick = function (event) {
  if (ready == true){             // mode = running
      console.log('AI Navigation');
      window.location = "AIPage.html";
  }
}
searchNav.onclick = function (event) {
  if (ready == true){         //mode = running
      console.log('Search Navigation');
      window.location = "SMPage.html";
  }
}
// LOGGING
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
//if mode = running
var TableUpdateTime = 1000;

function updateLogTable(){
  document.getElementById('DataMode').innerHTML = Math.random().toFixed(2);
  document.getElementById('DataYaw').innerHTML = Math.random().toFixed(2);
  document.getElementById('DataPitch').innerHTML = Math.random().toFixed(2);
  document.getElementById('DataRoll').innerHTML = Math.random().toFixed(2);
  document.getElementById('DataSpeed').innerHTML = Math.random().toFixed(2);
  setTimeout(updateLogTable,TableUpdateTime);
}
updateLogTable();

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
      if (yawTarget <= 180 && yawTarget >= -180 && pitchTarget <= 19 && pitchTarget >= -19 && rollTarget <= 180 && rollTarget >= -180){
        console.log('Target Yaw = ' + yawTarget);
        console.log('Target Pitch = ' + pitchTarget);
        console.log('Target Roll = ' + rollTarget);

// AI Graphs
        var GraphUpdateTime = 100; // in ms
        var ElementsKept = 100;
        var ElementsCounted = 0;

        var YPRErrorGraph = $("#YPRErrorGraph");
          YPRErrorGraph.height = 300;
          YPRErrorGraph.width = 300;
        var YPRGraph = $("#YPRGraph");
        var AngVelGraph = $("#AngVelGraph");

        var commonOptions = {
          scales: {
            xAxes: [{
              type: 'time',
              time: {
                displayFormats: {
                  millisecond: 'mm:ss:SSS'
                }
              }
            }],
            yAxes: [{
              ticks: {
                beginAtZero: true
              }
            }]
          },
          legend: {display: false},
          tooltips:{enabled: false}
        };

        var YPRErrorChartInstant = new Chart(YPRErrorGraph,{
          type: 'line',
          data: {
            datasets: [{
              label: "YPR Error",
              data: 0,
            }]
          },
          options: Object.assign({}, commonOptions,{
            title:{
              display: true,
              text: "YPR Error",
              fontSize: 8
            }
          })
        });
        var YPRChartInstant = new Chart(YPRGraph,{
          type: 'line',
          data: {
            datasets: [{
              label: "YPR",
              data: 0,
            }]
          },
          options: Object.assign({}, commonOptions,{
            title:{
              display: true,
              text: "YPR",
              fontSize: 8
            }
          })
        });
        var AngVelChartInstant = new Chart(AngVelGraph,{
          type: 'line',
          data: {
            datasets: [{
              label: "Angular Velocity",
              data: 0,
            }]
          },
          options: Object.assign({}, commonOptions,{
            title:{
              display: true,
              text: "Angular Velocity",
              fontSize: 8
            }
          })
        });

        function addData(data) {
          if(data){
            YPRErrorChartInstant.data.labels.push(new Date());
            YPRErrorChartInstant.data.datasets.forEach((dataset) =>{dataset.data.push(data)});
            YPRChartInstant.data.labels.push(new Date());
            YPRChartInstant.data.datasets.forEach((dataset) =>{dataset.data.push(data)});
            AngVelChartInstant.data.labels.push(new Date());
            AngVelChartInstant.data.datasets.forEach((dataset) =>{dataset.data.push(data)});
            if(ElementsCounted > ElementsKept){
              YPRErrorChartInstant.data.labels.shift();
              YPRErrorChartInstant.data.datasets[0].data.shift();
              YPRChartInstant.data.labels.shift();
              YPRChartInstant.data.datasets[0].data.shift();
              AngVelChartInstant.data.labels.shift();
              AngVelChartInstant.data.datasets[0].data.shift();
            }
            else ElementsCounted++;
            YPRErrorChartInstant.update();
            YPRChartInstant.update();
            AngVelChartInstant.update();
          }
        };

        function updateAIGraphData(){
          data=Math.random();
          addData(data);
          setTimeout(updateAIGraphData,GraphUpdateTime);
        }
        updateAIGraphData();

      }
      else if (yawTarget > 180 || yawTarget < -180){
        alert('Target Yaw must be between -180\u00B0 and +180\u00B0');
      }
      else if (pitchTarget > 19 || pitchTarget < -19){
        alert('Target Pitch must be between -19\u00B0 and +19\u00B0');
      }
      else if (rollTarget > 180 || rollTarget < -180){
        alert('Target Roll must be between -180\u00B0 and +180\u00B0');
      }
      else{
        alert('Target Yaw must be between -180\u00B0 and +180\u00B0 \nTarget Pitch must be between -19\u00B0 and +19\u00B0 \nTarget Roll must be between -180\u00B0 and +180\u00B0');
      }
  }
}

// SM BUTTONS
if (searchmode){
  searchmode.onclick = function (event) {
      console.log('Search Mode');
      var GraphUpdateTime = 100; // in ms
      var ElementsKept = 100;
      var ElementsCounted = 0;

      var YPRGraph = $("#YPRGraph");
      var AngVelGraph = $("#AngVelGraph");

      var commonOptions = {
        scales: {
          xAxes: [{
            type: 'time',
            time: {
              displayFormats: {
                millisecond: 'mm:ss:SSS'
              }
            }
          }],
          yAxes: [{
            ticks: {
              beginAtZero: true
            }
          }]
        },
        legend: {display: false},
        tooltips:{enabled: false}
      };

      var YPRChartInstant = new Chart(YPRGraph,{
        type: 'line',
        data: {
          datasets: [{
            label: "YPR",
            data: 0,
          }]
        },
        options: Object.assign({}, commonOptions,{
          title:{
            display: true,
            text: "YPR",
            fontSize: 8
          }
        })
      });
      var AngVelChartInstant = new Chart(AngVelGraph,{
        type: 'line',
        data: {
          datasets: [{
            label: "Angular Velocity",
            data: 0,
          }]
        },
        options: Object.assign({}, commonOptions,{
          title:{
            display: true,
            text: "Angular Velocity",
            fontSize: 8
          }
        })
      });

      function addData(data) {
        if(data){
          YPRChartInstant.data.labels.push(new Date());
          YPRChartInstant.data.datasets.forEach((dataset) =>{dataset.data.push(data)});
          AngVelChartInstant.data.labels.push(new Date());
          AngVelChartInstant.data.datasets.forEach((dataset) =>{dataset.data.push(data)});
          if(ElementsCounted > ElementsKept){
            YPRChartInstant.data.labels.shift();
            YPRChartInstant.data.datasets[0].data.shift();
            AngVelChartInstant.data.labels.shift();
            AngVelChartInstant.data.datasets[0].data.shift();
          }
          else ElementsCounted++;
          YPRChartInstant.update();
          AngVelChartInstant.update();
        }
      };

      function updateSMGraphData(){
        data=Math.random();
        addData(data);
        setTimeout(updateSMGraphData,GraphUpdateTime);
      }
      updateSMGraphData();
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
