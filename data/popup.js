function checker() {
  var result = confirm('Are you sure?'); 
    if (result == false) {
      event.preventDefault();
    }
}

function checkerRestart() {
  var result = confirm('RESTART REQUIRED. Are you sure?');
    if (result == false) {
      event.preventDefault();
    }
}

function getURLParameter(sParam) {
  var sPageURL = window.location.search.substring(1);
  var sURLVariables = sPageURL.split('&');

  for (var i = 0; i < sURLVariables.length; i++) {

    var sParameterName = sURLVariables[i].split('=');

    if (sParameterName[0] == sParam) {
      return sParameterName[1];
    }
  }
}

if (getURLParameter("msg")) {
  document.getElementById("message").innerText = decodeURI(getURLParameter("msg"));
}