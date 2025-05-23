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

    // Mouse hold functionality
    actions.forEach(action => {
        const img = document.getElementById(action);
        let intervalId = null;

        img.onmousedown = () => {
            highlightButton(img);
            intervalId = holdCommand(commandMap[action]);
        };

        img.onmouseup = img.onmouseleave = () => {
            clearInterval(intervalId);
        };
    });

    // Keyboard input handling
    let intervalId = null;
    let lastKey = null;

    document.addEventListener("keydown", (e) => {
        const keyMap = {
            ArrowUp: ["up", 1],
            ArrowDown: ["down", 2],
            ArrowLeft: ["left", 3],
            ArrowRight: ["right", 4],
            " ": ["stop", 0]
        };

        const action = keyMap[e.key];
        if (action && e.key !== lastKey) {
            lastKey = e.key;
            const [id, cmd] = action;
            const btn = document.getElementById(id);
            highlightButton(btn);
            sendCommand(cmd);
            if (cmd !== 0) {
                intervalId = setInterval(() => sendCommand(cmd), 300);
            }
        }
    });

    document.addEventListener("keyup", () => {
        clearInterval(intervalId);
        lastKey = null;
    });
};

// Visual flash for button
function highlightButton(btn) {
    if (!btn) return;
    btn.classList.add("active");
    setTimeout(() => {
        btn.classList.remove("active");
    }, 200);
}
