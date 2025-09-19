import React, { useEffect, useState, useRef } from "react";
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  Legend,
  ResponsiveContainer,
  ReferenceLine,
} from "recharts";

export default function SteerChart() {
  const [data, setData] = useState([]);
  const startTimeRef = useRef(Date.now());

  useEffect(() => {
    const host = process.env.REACT_APP_WS_HOST || "localhost";
    const port = process.env.REACT_APP_WS_PORT || "5000";
    const ws = new WebSocket(`ws://${host}:${port}`);

    ws.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data);
        const now = Date.now();
        const elapsedSec = (now - startTimeRef.current) / 1000;

        // type이 chart일 때만 처리
        if (message.type === 'chart' && message.data) {
          setData((prev) => {
            const last = prev[prev.length - 1] || {};

            const merged = {
              time: elapsedSec,
              cmd_steer:
                message.data.cmd_steer !== undefined
                  ? message.data.cmd_steer
                  : last.cmd_steer,
            };

            const updated = [...prev, merged];
            // 50초 윈도우 유지
            return updated.filter((point) => elapsedSec - point.time <= 50);
          });
        } else if (!message.type) {
          // 기존 방식 호환성 (type이 없는 경우)
          setData((prev) => {
            const last = prev[prev.length - 1] || {};

            const merged = {
              time: elapsedSec,
              cmd_steer:
                message.cmd_steer !== undefined
                  ? message.cmd_steer
                  : last.cmd_steer,
            };

            const updated = [...prev, merged];
            return updated.filter((point) => elapsedSec - point.time <= 50);
          });
        }
      } catch (error) {
        console.error('SteerChart WebSocket parse error:', error);
      }
    };

    ws.onopen = () => {
      console.log("SteerChart WebSocket connected");
    };

    ws.onerror = (error) => {
      console.error("SteerChart WebSocket error:", error);
    }; 

    return () => ws.close();
  }, []);

  return (
    <div style={{ width: "100%", height: "100%" }}>
      <ResponsiveContainer width="100%" height="100%">
        <LineChart data={data}>
          <CartesianGrid strokeDasharray="3 3" />
          <XAxis
            dataKey="time"
            type="number"
            domain={["dataMin", "dataMax"]}
            tickFormatter={(t) => t.toFixed(1) + "s"}
            axisLine={true}
            tickLine={true}
          />
          <YAxis
            domain={[0, "dataMax"]}
            tickFormatter={(value) => value.toFixed(2)}
          />
          <ReferenceLine y={0} stroke="#000" strokeWidth={1} />
          <Tooltip
            labelFormatter={(label) => `${label.toFixed(2)} sec`}
            formatter={(value, name) => [value.toFixed(2), name]}
          />
          <Legend />
          <Line
            type="linear"
            dataKey="cmd_steer"
            stroke="#D62728"
            dot={false}
            name="Command Steer"
            isAnimationActive={false}
          />
          <Line
            type="linear"
            dataKey="current_steer"
            stroke="#1F77B4"
            dot={false}
            name="Current Steer"
            isAnimationActive={false}
          />
        </LineChart>
      </ResponsiveContainer>
    </div>
  );
}
