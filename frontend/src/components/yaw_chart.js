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
    const ws = new WebSocket("ws://192.168.0.34:5000");  // TODO: ip 주소 변경

    ws.onmessage = (event) => {
      const message = JSON.parse(event.data); // 통합 JSON
      const now = Date.now();
      const elapsedSec = (now - startTimeRef.current) / 1000;

      setData((prev) => {
        const last = prev[prev.length - 1] || {};

        // yaw 값이 없으면 이전 값 유지
        const merged = {
          time: elapsedSec,
          raw: message.yaw !== undefined ? message.yaw : last.raw,
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
          {/* Yaw 범위: -π ~ π */}
          <YAxis domain={[-3.2, 3.2]} />
          <Tooltip
            labelFormatter={(label) => `${label.toFixed(2)} sec`}
            formatter={(value) => [value.toFixed(3), "Yaw"]}
          />
          <Legend />
          <Line
            type="linear"
            dataKey="raw"
            stroke="#1F77B4"
            dot={false}
            name="Yaw"
            isAnimationActive={false} // 실시간 반응 빠르게
          />
        </LineChart>
      </ResponsiveContainer>
    </div>
  );
}
