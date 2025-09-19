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

            // 새 값이 있으면 업데이트, 없으면 이전 값 유지
            const merged = {
              time: elapsedSec,
              cmd_speed:
                message.data.cmd_speed !== undefined ? message.data.cmd_speed : last.cmd_speed,
              current_speed:
                message.data.current_speed !== undefined
                  ? message.data.current_speed
                  : last.current_speed,
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
              cmd_speed:
                message.cmd_speed !== undefined ? message.cmd_speed : last.cmd_speed,
              current_speed:
                message.current_speed !== undefined
                  ? message.current_speed
                  : last.current_speed,
            };

            const updated = [...prev, merged];
            return updated.filter((point) => elapsedSec - point.time <= 50);
          });
        }
      } catch (error) {
        console.error('SpeedChart WebSocket parse error:', error);
      }
    };

    ws.onopen = () => {
      console.log("SpeedChart WebSocket connected");
    };

    ws.onerror = (error) => {
      console.error("SpeedChart WebSocket error:", error);
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
            stroke="#D62728"
            dot={false}
            name="Command Speed"
            isAnimationActive={false}
          />
          <Line
            type="linear"
            dataKey="current_speed"
            stroke="#1F77B4"
            dot={false}
            name="Current Speed"
            isAnimationActive={false}
          />
        </LineChart>
      </ResponsiveContainer>
    </div>
  );
}
