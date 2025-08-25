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

export default function YawChart() {
  const [data, setData] = useState([]);
  const startTimeRef = useRef(Date.now());

  useEffect(() => {
    const ws = new WebSocket("ws://localhost:5000");
    ws.onmessage = (event) => {
      const yaw = JSON.parse(event.data);
      const now = Date.now();
      const elapsedSec = (now - startTimeRef.current) / 1000;

      setData((prev) => {
        // 새 데이터 추가
        const updated = [...prev, { time: elapsedSec, raw: yaw.raw }];

        // 50초 버퍼 유지
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
          <YAxis domain={[-3.2, 3.2]} /> {/* -pi ~ +pi */}
          <Tooltip
            labelFormatter={(label) => `${label.toFixed(2)} sec`}
          />
          <Legend />
          <Line
            type="monotone"
            dataKey="raw"
            stroke="#1F77B4"
            dot={false}
            name="Global Yaw"
            isAnimationActive={false} // 실시간 반응 빠르게
          />
        </LineChart>
      </ResponsiveContainer>
    </div>
  );
}
