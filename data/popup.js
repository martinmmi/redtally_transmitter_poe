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