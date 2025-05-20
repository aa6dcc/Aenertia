function sayHello() {
  alert("Hi there! This is a JS alert.");
}

function flashLED() {
  fetch('/flash-led')
    .then(response => response.text())
    .then(data => alert(data))
    .catch(error => alert("Failed to flash LED: " + error));
}
