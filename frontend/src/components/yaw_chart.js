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

            // yaw 값이 없으면 이전 값 유지
            const merged = {
              time: elapsedSec,
              raw: message.data.yaw !== undefined ? message.data.yaw : last.raw,
            };

            const updated = [...prev, merged];
            // 최근 50초 데이터만 유지
            return updated.filter((point) => elapsedSec - point.time <= 50);
          });
        } else if (!message.type) {
          // 기존 방식 호환성 (type이 없는 경우)
          setData((prev) => {
            const last = prev[prev.length - 1] || {};

            const merged = {
              time: elapsedSec,
              raw: message.yaw !== undefined ? message.yaw : last.raw,
            };

            const updated = [...prev, merged];
            return updated.filter((point) => elapsedSec - point.time <= 50);
          });
        }
      } catch (error) {
        console.error('YawChart WebSocket parse error:', error);
      }
    };

    ws.onopen = () => {
      console.log("YawChart WebSocket connected");
    };

    ws.onerror = (error) => {
      console.error("YawChart WebSocket error:", error);
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
