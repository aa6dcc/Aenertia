function sayHello() {
  alert("Hi there! This is a JS alert.");
}

function flashLED() {
  fetch('/flash-led')
    .then(response => response.text())
    .then(data => alert(data))
    .catch(error => alert("Failed to flash LED: " + error));
}

let currentTab = 'pidTab';
let isDirty = false;

// Mark as dirty when any input changes
document.addEventListener('input', () => {
  isDirty = true;
});

function showTab(tabId) {
  if (tabId === currentTab) return;

  if (isDirty && !confirm("You have unsaved changes. Are you sure you want to switch tabs?")) {
    return;
  }

  document.querySelectorAll('.tab-content').forEach(tab => tab.classList.remove('active'));
  document.getElementById(tabId).classList.add('active');
  currentTab = tabId;
  isDirty = false;
}

// Populate PID tables with previously submitted values
function populatePIDTables() {
  fetch('/pid-values')
    .then(res => res.json())
    .then(data => {
      ['inner', 'outer'].forEach(loop => {
        const table = document.getElementById(loop + 'Table');
        data[loop].forEach(rowValues => {
          const row = table.insertRow(-1);
          rowValues.forEach(val => row.insertCell().textContent = val);
        });
      });
    });
}

// Call populate on page load
document.addEventListener('DOMContentLoaded', () => {
  populatePIDTables();
});
