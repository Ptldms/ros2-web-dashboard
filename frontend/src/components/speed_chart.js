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
} from "recharts";

export default function SpeedChart() {
  const [data, setData] = useState([]);
  const startTimeRef = useRef(Date.now());

  useEffect(() => {
    const ws = new WebSocket("ws://localhost:5000");

    ws.onmessage = (event) => {
      const message = JSON.parse(event.data); // 통합 JSON
      const now = Date.now();
      const elapsedSec = (now - startTimeRef.current) / 1000;

      setData((prev) => {
        const last = prev[prev.length - 1] || {};

        // 새 값이 있으면 업데이트, 없으면 이전 값 유지
        const merged = {
          time: elapsedSec,
          cmd_speed:
            message.cmd_speed !== undefined ? message.cmd_speed : last.cmd_speed,
          current_speed:
            message.current_speed !== undefined
              ? message.current_speed
              : last.current_speed,
        };

        const updated = [...prev, merged];
        // 최근 50초 데이터만 유지
        return updated.filter((point) => elapsedSec - point.time <= 50);
      });
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
          />
          {/* 속도는 0 이상 */}
          <YAxis domain={[0, "dataMax"]} />
          <Tooltip
            labelFormatter={(label) => `${label.toFixed(2)} sec`}
            formatter={(value, name) => [value.toFixed(2), name]}
          />
          <Legend />
          <Line
            type="linear"
            dataKey="cmd_speed"
            stroke="#FF6B6B"
            dot={false}
            name="Command Speed"
            isAnimationActive={false}
          />
          <Line
            type="linear"
            dataKey="current_speed"
            stroke="#4ECDC4"
            dot={false}
            name="Current Speed"
            isAnimationActive={false}
          />
        </LineChart>
      </ResponsiveContainer>
    </div>
  );
}
