const WebSocket = require("ws");

const wss = new WebSocket.Server({ host: "0.0.0.0", port: 5000 });

function heartbeat() {
    this.isAlive = true;
}

wss.on("connection", (ws) => {
    console.log("Client connected");
    ws.isAlive = true;
    ws.on("pong", heartbeat);

    ws.on("message", (message) => {
        try {
            // 메시지 유효성 검사
            const data = JSON.parse(message);
            console.log("Received data:", data);
            
            // 모든 클라이언트에게 브로드캐스트
            wss.clients.forEach((client) => {
                if (client !== ws && client.readyState === WebSocket.OPEN) {
                    try {
                        client.send(message.toString());
                    } catch (err) {
                        console.error("Failed to send message to client:", err);
                    }
                }
            });
        } catch (err) {
            console.error("Invalid JSON message:", err);
        }
    });

    ws.on("close", () => {
        console.log("Client disconnected");
    });

    ws.on("error", (err) => {
        console.error("Client WebSocket error:", err);
    });
});

// Heartbeat 체크 (30초는 적절함)
const interval = setInterval(() => {
    wss.clients.forEach((ws) => {
        if (!ws.isAlive) {
            console.log("Terminating dead connection");
            return ws.terminate();
        }
        ws.isAlive = false;
        ws.ping();
    });
}, 30000);

wss.on("error", (err) => {
    console.error("WebSocket server error:", err);
});

// 서버 종료 시 정리
process.on('SIGINT', () => {
    console.log("Shutting down WebSocket server...");
    clearInterval(interval);
    
    // 즉시 종료
    wss.clients.forEach((ws) => ws.terminate());
    wss.close();
    
    console.log("WebSocket server closed");
    process.exit(0);
});

console.log("WebSocket server running on ws://0.0.0.0:5000");