const WebSocket = require("ws");

// 5000 Port 사용
const wss = new WebSocket.Server({ host: "0.0.0.0", port: 5000 });

wss.on("connection", (ws) => {
    console.log("Client connected");
    ws.on("message", (message) => {
        // ros2_bridge/yaw_node.py에서 받은 yaw 데이터 -> 모든 클라이언트에 broadcast
        wss.clients.forEach((client) => {
            if(client.readyState === WebSocket.OPEN) {
                client.send(message.toString());
            }
        });
    });
});

console.log("WebSocket server running on ws://0.0.0.0:5000");