const WebSocket = require("ws");

// 5000 Port 사용
const wss = new WebSocket.Server({ host: "0.0.0.0", port: 5000 });

wss.on("connection", (ws) => {
    console.log("Client connected");

    ws.on("message", (message) => {
        // 모든 클라이언트에 broadcast, 오류 방지
        wss.clients.forEach((client) => {
            if (client.readyState === WebSocket.OPEN) {
                try {
                    client.send(message.toString());
                } catch (err) {
                    console.error("Failed to send message to client:", err);
                }
            }
        });
    });

    ws.on("error", (err) => {
        console.error("WebSocket client error:", err);
    });

    ws.on("close", () => {
        console.log("Client disconnected");
    });
});

wss.on("error", (err) => {
    console.error("WebSocket server error:", err);
});

console.log("WebSocket server running on ws://0.0.0.0:5000");
