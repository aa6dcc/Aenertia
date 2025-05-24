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

    // Mouse control
    actions.forEach(action => {
        const btn = document.getElementById(action);
        let intervalId = null;

        btn.onmousedown = () => {
            setActive(btn);
            intervalId = holdCommand(commandMap[action]);
        };

        btn.onmouseup = btn.onmouseleave = () => {
            btn.classList.remove("active");
            clearInterval(intervalId);
        };
    });

    // Keyboard control
    let keyIntervalId = null;
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
            setActive(btn);
            sendCommand(cmd);
            if (cmd !== 0) {
                keyIntervalId = setInterval(() => sendCommand(cmd), 300);
            }
        }
    });

    document.addEventListener("keyup", () => {
        clearInterval(keyIntervalId);
        clearActive();
        lastKey = null;
    });
};

// only set active if not already
function setActive(btn) {
    if (!btn.classList.contains("active")) {
        clearActive();
        btn.classList.add("active");
    }
}

function clearActive() {
    document.querySelectorAll(".arrow.active").forEach(el => el.classList.remove("active"));
}
