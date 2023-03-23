function checker() {
  var result = confirm('Are you sure?'); 
    if (result == false) {
      event.preventDefault();
    }
}

function checkerRestart() {
  var result = confirm('RESTART REQUIRED. Set the values and send a control restart message to receivers. Are you sure?');
    if (result == false) {
      event.preventDefault();
    }
}

function checkerWlan() {
  var result = confirm('Are the WLAN CREDENTIALS right?'); 
    if (result == false) {
      event.preventDefault();
    }
}

function toggle(){
  var blur = document.getElementById('blur');
  blur.classList.toggle('active')
  var popup = document.getElementById('popup');
  popup.classList.toggle('active')
}