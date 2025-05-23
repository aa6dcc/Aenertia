function sendCommand(command) {
    fetch(`/led/${command}`);
}

function holdCommand(command) {
    sendCommand(command);
    return setInterval(() => sendCommand(command), 300);
}

window.onload = () => {
    const actions = ["up", "down", "left", "right", "stop"];
    const commandMap = {
        up: 1,
        down: 2,
        left: 3,
        right: 4,
        stop: 0
    };

    actions.forEach(action => {
        const img = document.getElementById(action);
        let intervalId = null;

        img.onmousedown = () => {
            intervalId = holdCommand(commandMap[action]);
        };

        img.onmouseup = img.onmouseleave = () => {
            clearInterval(intervalId);
        };
    });
};
