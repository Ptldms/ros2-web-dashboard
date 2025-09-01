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

export default function SteerChart() {
  const [data, setData] = useState([]);
  const startTimeRef = useRef(Date.now());

  useEffect(() => {
    const ws = new WebSocket("ws://localhost:5000");
    ws.onmessage = (event) => {
        const steerData = JSON.parse(event.data);
        const now = Date.now();
        const elapsedSec = (now - startTimeRef.current) / 1000;

        setData((prev) => {
            const last = prev[prev.length - 1] || {};
            const merged = {
                time: elapsedSec,
                cmd_steer: steerData.cmd_steer !== undefined ? steerData.cmd_steer : last.cmd_steer,
                // current_steer: steerData.current_steer !== undefined ? steerData.current_steer : last.current_steer,
            };

            const updated = [...prev, merged];
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
          <YAxis 
            domain={[0, "dataMax"]} 
            tickFormatter={(value) => value.toFixed(2)}
          />
          <Tooltip
            labelFormatter={(label) => `${label.toFixed(2)} sec`}
            formatter={(value, name) => [value.toFixed(2), name]}
          />
          <Legend />
          <Line
            type="monotone"
            dataKey="cmd_steer"
            stroke="#FF6B6B"
            dot={false}
            name="Command steer"
            isAnimationActive={false}
          />
          <Line
            type="monotone"
            dataKey="current_steer"
            stroke="#4ECDC4"
            dot={false}
            name="Current steer"
            isAnimationActive={false}
          />
        </LineChart>
      </ResponsiveContainer>
    </div>
  );
}