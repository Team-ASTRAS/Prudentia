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

// AI BUTTONS
if (go){
  go.onclick = function (event) {
      yawTarget = document.getElementById("YawTarget").value;
      pitchTarget = document.getElementById("PitchTarget").value;
      rollTarget = document.getElementById("RollTarget").value;
      if (yawTarget <= 180 && yawTarget >= -180 && pitchTarget <= 19 && pitchTarget >= -19 && rollTarget <= 180 && rollTarget >= -180){
        console.log('Target Yaw = ' + yawTarget); // Send Target to Python Script
        console.log('Target Pitch = ' + pitchTarget);
        console.log('Target Roll = ' + rollTarget);
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
};

// SM BUTTONS
if (searchmode){
  searchmode.onclick = function (event) {
      console.log('Search Mode'); // Send mode to python
  }
};

// GRAPHS
function updateGraphs(mode, routine, position, velocityMag, error) {
    var ElementsKept = 50;
    var ElementsCounted = 0;
    var commonOptions = {
      responsive: true,
      maintainAspectRatio: false,
      legend: {display: false},
      tooltips:{enabled: false},
      elements: {
                point:{
                    radius: 0
                }
            }
    };
    var YPRGraph = $("#YPRGraph");
    var AngVelGraph = $("#AngVelGraph");

    var YPRChartInstant = new Chart(YPRGraph,{
      type: 'line',
      data: {
        datasets: [{
          label: "Yaw",
          data: 0,
          borderColor: ['rgb(255,0,0)'],
          borderWidth: 1,
          fill: false},
        {label: "Pitch",
          data: 0,
          borderColor: ['rgb(0,255,0)'],
          borderWidth: 1,
          fill: false},
        {label: "Roll",
          data: 0,
          borderColor: ['rgb(0,0,255)'],
          borderWidth: 1,
          fill: false}
      ]
      },
      options: Object.assign({}, commonOptions,{
        title:{
          display: true,
          text: "YPR",
          fontSize: 18
        },
        scales: {
          xAxes: [{
            type: 'time',
            scaleLabel: {
              display: true,
              labelString: 'time (min:sec)',
            },
            time: {
              displayFormats: {
                second: 'mm:ss'
              }
            }
          }],
          yAxes: [{
            ticks: {
              beginAtZero: true
            },
            scaleLabel: {
              display: true,
              labelString: 'Degrees'
            }
          }]
        }
      })
    });

    var AngVelChartInstant = new Chart(AngVelGraph,{
      type: 'line',
      data: {
        datasets: [{
          label: "Angular Velocity",
          data: 0,
          borderColor: ['rgb(0,255,0)'],
          borderWidth: 1,
          fill: false
        }]
      },
      options: Object.assign({}, commonOptions,{
        title:{
          display: true,
          text: "Angular Velocity",
          fontSize: 18
        },
        scales: {
          xAxes: [{
            type: 'time',
            scaleLabel: {
              display: true,
              labelString: 'time (min:sec)',
            },
            time: {
              displayFormats: {
                second: 'mm:ss'
              }
            }
          }],
          yAxes: [{
            ticks: {
              beginAtZero: true
            },
            scaleLabel: {
              display: true,
              labelString: 'Degrees/Sec'
            }
          }]
        }
      })
    });

    YPRChartInstant.data.labels.push(new Date());
    YPRChartInstant.data.datasets[0].data.push(position[0].toFixed(2));
    YPRChartInstant.data.datasets[1].data.push(position[1].toFixed(2));
    YPRChartInstant.data.datasets[2].data.push(position[2].toFixed(2));
    AngVelChartInstant.data.labels.push(new Date());
    AngVelChartInstant.data.datasets[0].data.push(velocityMag.toFixed(2));

    if (routine = "AttitudeInput"){
      var YPRErrorGraph = $("#YPRErrorGraph");

      var YPRErrorChartInstant = new Chart(YPRErrorGraph,{
        type: 'line',
        data: {
          datasets: [{
            label: "Yaw Error",
            data: 0,
            borderColor: ['rgb(255,0,0)'],
            borderWidth: 1,
            fill: false},
          {label: "Pitch Error",
            data: 0,
            borderColor: ['rgb(0,255,0)'],
            borderWidth: 1,
            fill: false},
          {label: "Roll Error",
            data: 0,
            borderColor: ['rgb(0,0,255)'],
            borderWidth: 1,
            fill: false}
        ]
        },
        options: Object.assign({}, commonOptions,{
          title:{
            display: true,
            text: "YPR Error",
            fontSize: 18
          },
          scales: {
            xAxes: [{
              type: 'time',
              scaleLabel: {
                display: true,
                labelString: 'time (min:sec)',
              },
              time: {
                displayFormats: {
                  second: 'mm:ss'
                }
              }
            }],
            yAxes: [{
              ticks: {
                beginAtZero: true
              },
              scaleLabel: {
                display: true,
                labelString: 'Degrees'
              }
            }]
          }
      })
    });

      YPRErrorChartInstant.data.labels.push(new Date());
      YPRErrorChartInstant.data.datasets[0].data.push(error[0]);
      YPRErrorChartInstant.data.datasets[1].data.push(error[1]);
      YPRErrorChartInstant.data.datasets[2].data.push(error[2]);
      if(ElementsCounted > ElementsKept){
          YPRErrorChartInstant.data.labels.shift();
          YPRErrorChartInstant.data.datasets[0].data.shift();
          YPRErrorChartInstant.data.datasets[1].data.shift();
          YPRErrorChartInstant.data.datasets[2].data.shift();
          YPRChartInstant.data.labels.shift();
          YPRChartInstant.data.datasets[0].data.shift();
          YPRChartInstant.data.datasets[1].data.shift();
          YPRChartInstant.data.datasets[2].data.shift();
          AngVelChartInstant.data.labels.shift();
          AngVelChartInstant.data.datasets[0].data.shift();
        }
        else ElementsCounted++;
        YPRErrorChartInstant.update();
        YPRChartInstant.update();
        AngVelChartInstant.update();
      }

    else if (routine = "SearchMode"){
      if(ElementsCounted > ElementsKept){
          YPRChartInstant.data.labels.shift();
          YPRChartInstant.data.datasets[0].data.shift();
          YPRChartInstant.data.datasets[1].data.shift();
          YPRChartInstant.data.datasets[2].data.shift();
          AngVelChartInstant.data.labels.shift();
          AngVelChartInstant.data.datasets[0].data.shift();
      }
      else ElementsCounted++;
      YPRChartInstant.update();
      AngVelChartInstant.update();
    }
};

function setState(state){
    var msg = {"messageType":"setState", "state":state}
    websocket.send(JSON.stringify(msg));
};

function updateLogTable(mode, routine, position, velocityMag) {
    modeName = mode
    if (mode == "running") {
        modeName = modeName + " (" + routine + ")"
    }
    document.getElementById('DataMode').innerHTML = modeName;
    document.getElementById('DataYaw').innerHTML = position[0].toFixed(2);
    document.getElementById('DataPitch').innerHTML = position[1].toFixed(2);
    document.getElementById('DataRoll').innerHTML = position[2].toFixed(2);
    document.getElementById('DataSpeed').innerHTML = velocityMag.toFixed(2);
};

function start(websocketServerLocation){
    //Attempt connection
    websocket = new WebSocket(websocketServerLocation);

    //Right now, we assume the only message Prudentia sends is it's sharedData
    //This assumption can be changed later on if necessary
    websocket.onmessage = function (event) {
        data = JSON.parse(event.data);

        updateLogTable(data["state"], data["routine"], data["position"], data["velocityMag"])
        updateGraphs(data["state"], data["routine"], data["position"], data["velocityMag"], data["error"])
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
};

function refreshData(){
    if (websocket.readyState === websocket.OPEN){
        console.log("Refreshing data")
        //Server returns data when the "messageType" field is "getData"
        var msg = {"messageType":"getData"}
        websocket.send(JSON.stringify(msg));
    }
    //Repeat this function again later
    setTimeout(refreshData, dataUpdate);
};
