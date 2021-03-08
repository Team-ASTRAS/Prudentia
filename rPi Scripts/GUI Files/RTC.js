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
