var script1 = document.createElement('script');
script1.src = 'https://ajax.googleapis.com/ajax/libs/jquery/1.4.1/jquery.js';
script1.type = 'text/javascript';
document.getElementsByTagName('head')[0].appendChild(script1);
var script2 = document.createElement('script');
script2.src = 'https://cdnjs.cloudflare.com/ajax/libs/Chart.js/2.7.2/Chart.bundle.js';
script2.type = 'text/javascript';
document.getElementsByTagName('head')[0].appendChild(script2);

var dataMode = document.querySelector('.DataMode'),
    dataRoutine = document.querySelector('.DataRoutine'),
    dataYaw = document.querySelector('.DataYaw'),
    dataPitch = document.querySelector('.DataPitch'),
    dataRoll = document.querySelector('.DataRoll'),
    dataSpeed = document.querySelector('.DataSpeed'),
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
    searchmode = document.querySelector('.SearchButton');

//Connect to RPi here
ip = "ws://172.30.0.20:8010/";
start(ip);

dataUpdate =  100;
refreshData() //This function is called recursively

// Button callbacks
// SHUTDOWN, STABILIZE, STOP BUTTONS
shutdown.onclick = function (event) {
    var quit = confirm('Did you remember to save your data? Click CANCEL to go back and save your data. Click OK to continue the shut down process. ');
    if (quit == true){
      alert('Hold Prudentia steady while the motors spin down. Click OK to begin spin down.');
      console.log('Shutdown');
      setState("shutdown");
    }
}
stabilize.onclick = function (event) {
  if (ready == true){           // Change to if mode = running
    console.log('Stabilize');
    setRoutine("stabilize");
  }
}
if (calibrate){
  calibrate.onclick = function (event) {
      alert('Align Prudentia with the calibration target. Click OK when Prudentia is aligned.');
      calibrated = true;
      ready = true;
      console.log('System Calibrated');
  }
}
stop.onclick = function (event) {
    console.log('Stop');
    stop.style.background = "#cf0404";
    setState("standby");
}
// NAVIGATION BUTTONS
homeNav.onclick = function (event) {
    console.log('Home Navigation');
    window.location = "Homepage.html";
    Initialized = false;
}
instructionsNav.onclick = function (event) {
    console.log('Instructions Navigation');
    window.location = "InstructionPage.html";
    Initialized = false;
}
aiNav.onclick = function (event) {
  if (ready == true){             // mode = running
      console.log('AI Navigation');
      window.location = "AIPage.html";
      Initialized = false;
  }
}
searchNav.onclick = function (event) {
  if (ready == true){         //mode = running
      console.log('Search Navigation');
      window.location = "SMPage.html";
      Initialized = false;
  }
}
// LOGGING
if (startlog){
    startlog.onclick = function (event) {
        startlog.style.color = "#36d146";
        var msg = {"messageType":"DataLogging", "LogType":"StartLog"};
        jsonMSG = JSON.stringify(msg);
        console.log(jsonMSG);
        websocket.send(jsonMSG);
      }
      stoplog.onclick = function (event) {
        startlog.style.color = "#F7F7F7";
        var msg = {"messageType":"DataLogging", "LogType":"StopLog"};
        jsonMSG = JSON.stringify(msg);
        console.log(jsonMSG);
        websocket.send(jsonMSG);
      }
      clearlog.onclick = function (event) {
        console.log('Clear Log');
        var msg = {"messageType":"DataLogging", "LogType":"ClearLog"};
        jsonMSG = JSON.stringify(msg);
        websocket.send(jsonMSG);
      }
      downloadlog.onclick = function (event) {
        console.log('Download Log');
        filename = prompt("Filename:");
        if (filename == null || filename == ""){
          filename = "Data";
          }
          var msg = { "messageType": "downloadData", "filename" : filename};
          jsonMSG = JSON.stringify(msg);
          websocket.send(jsonMSG);
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
        console.log('Target Yaw = ' + yawTarget);
        console.log('Target Pitch = ' + pitchTarget);
        console.log('Target Roll = ' + rollTarget);

        Target = [parseInt(yawTarget),parseInt(pitchTarget),parseInt(rollTarget)];
        var msg = {"messageType":"setTarget", "target":Target};
        jsonMSG = JSON.stringify(msg);
        console.log(jsonMSG);
        websocket.send(jsonMSG);
        setRoutine("attitudeInput");
        page = "AI";
        GraphInitialization();
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
      console.log('Search Mode');
      page = "SM";
      GraphInitialization();
      setRoutine("searchMode");
      document.getElementById("Camera").src = 'http://172.30.135.229:8000/';
  }
};

Initialized = true;
// GRAPHS
function GraphInitialization() {
  Initialized = true;
  ElementsKept = 50;
  ElementsCounted = 0;

  commonOptions = {
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

  YPRGraph = $("#YPRGraph");
  AngVelGraph = $("#AngVelGraph");

  YPRChartInstant = new Chart(YPRGraph,{
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

  AngVelChartInstant = new Chart(AngVelGraph,{
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

  if (page == "AI"){
    YPRErrorGraph = $("#YPRErrorGraph");

    YPRErrorChartInstant = new Chart(YPRErrorGraph,{
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
  })
};

};
function updateGraphs(routine, orientation, velocityMag, eulerError, image) {
  if (Initialized == true) {
    YPRChartInstant.data.labels.push(new Date());
    YPRChartInstant.data.datasets[0].data.push(orientation[0].toFixed(2));
    YPRChartInstant.data.datasets[1].data.push(orientation[1].toFixed(2));
    YPRChartInstant.data.datasets[2].data.push(orientation[2].toFixed(2));
    AngVelChartInstant.data.labels.push(new Date());
    AngVelChartInstant.data.datasets[0].data.push(velocityMag.toFixed(2));

    if (routine == "attitudeInput"){
      YPRErrorChartInstant.data.labels.push(new Date());
      YPRErrorChartInstant.data.datasets[0].data.push(eulerError[0]);
      YPRErrorChartInstant.data.datasets[1].data.push(eulerError[1]);
      YPRErrorChartInstant.data.datasets[2].data.push(eulerError[2]);
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

      else if (routine == "search"){
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
  }
};

function setState(state){
    var msg = {"messageType":"setState", "state":state}
    websocket.send(JSON.stringify(msg));
};

function setRoutine(routine) {
    var msg = { "messageType": "setRoutine", "routine": routine }
    websocket.send(JSON.stringify(msg));
}

function updateLogTable(mode, routine, orientation, velocityMag) {
    document.getElementById('DataMode').innerHTML = mode;
    document.getElementById('DataRoutine').innerHTML = routine;
    document.getElementById('DataYaw').innerHTML = orientation[0].toFixed(2);
    document.getElementById('DataPitch').innerHTML = orientation[1].toFixed(2);
    document.getElementById('DataRoll').innerHTML = orientation[2].toFixed(2);
    document.getElementById('DataSpeed').innerHTML = velocityMag.toFixed(2);
}

function start(websocketServerLocation){
    //Attempt connection
    websocket = new WebSocket(websocketServerLocation);

    //Right now, we assume the only message Prudentia sends is it's sharedData
    //This assumption can be changed later on if necessary
    websocket.onmessage = function (event) {
        data = JSON.parse(event.data);

        if (data.hasOwnProperty("csvFile")) {

            download(data["filename"] + ".csv", data["csvFile"])
        }
        else {
            updateLogTable(data["state"], data["routine"], data["orientation"], data["velocityMag"])

            updateGraphs(data["routine"], data["orientation"], data["velocityMag"], data["eulerError"], data["image"])
        }

    };

    //If an error occurs, close socket. This will call websocket.onclose
    websocket.onerror = function(error){
        console.error("WebSocket error: Closing socket.");
        websocket.close();
    }

    //Retry connection after 2 seconds when socket closes
    websocket.onclose = function(event){
        retryTime = 2000;
        console.log("Websocket closed: ", event, "Reconnecting in ", retryTime, " ms.")
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

function download(filename, text) {
    var element = document.createElement('a');
    element.setAttribute('href', 'data:text/plain;charset=utf-8,' + encodeURIComponent(text));
    element.setAttribute('download', filename);

    element.style.display = 'none';
    document.body.appendChild(element);

    element.click();

    document.body.removeChild(element);
}
