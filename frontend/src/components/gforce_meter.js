import React, { useEffect, useState } from "react";

export default function GForceMeter() {
  const [gForce, setGForce] = useState({ g_longitudinal: 0, g_lateral: 0 });
  const G_TO_MS2 = 9.81;

  useEffect(() => {
    const host = process.env.REACT_APP_WS_HOST || "localhost";
    const port = process.env.REACT_APP_WS_PORT || "5000";
    const ws = new WebSocket(`ws://${host}:${port}`);


    ws.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data);

        if (message.type === 'chart' && message.data) {
          const { g_longitudinal, g_lateral } = message.data;
          if (g_longitudinal !== undefined && g_lateral !== undefined) {
            setGForce({
              g_longitudinal: g_longitudinal * G_TO_MS2,
              g_lateral: g_lateral * G_TO_MS2,
            });
          }
        } else if (!message.type) {
          if (message.g_longitudinal !== undefined && message.g_lateral !== undefined) {
            setGForce({
              g_longitudinal: message.g_longitudinal * G_TO_MS2,
              g_lateral: message.g_lateral * G_TO_MS2,
            });
          }
        }
      } catch (error) {
        console.error('GForceMeter WebSocket parse error:', error);
      }
    };

    ws.onopen = () => {
      console.log("GForceMeter WebSocket connected");
    };

    ws.onerror = (error) => {
      console.error("GForceMeter WebSocket error:", error);
    };

    return () => ws.close();
  }, []);

  const size = 220;
  const center = size / 2;

  // 실제 m/s² 단위로 원 범위 설정
  const innerLimit = 1.0;
  const middleLimit = 3.0;
  const outerLimit = 5.0;


  const outerRadius = center - 5; // 바깥 원 반지름
  const middleRadius = (middleLimit / outerLimit) * outerRadius; // 중간 원
  const innerRadius = (innerLimit / outerLimit) * outerRadius; // 안쪽 원

  const scale = outerRadius / outerLimit;

  // 좌표 계산 (앞뒤 G = Y축, 좌우 G = X축)
  const x = center + gForce.g_lateral * scale;
  const y = center - gForce.g_longitudinal * scale;

  return (
    <div
      style={{
        width: size + 40,
        padding: "0px",
        margin: "0 auto",
        textAlign: "center",
      }}
    >
      <svg width={size} height={size} style={{ background: "#f8f9fa", borderRadius: "50%" }}>
        {/* 안쪽 원 */}
        <circle cx={center} cy={center} r={innerRadius} stroke="#eee" strokeWidth="2" fill="none" />
        {/* 중간 원 */}
        <circle cx={center} cy={center} r={middleRadius} stroke="#ddd" strokeWidth="2" fill="none" />
        {/* 바깥 원 */}
        <circle cx={center} cy={center} r={outerRadius} stroke="#ccc" strokeWidth="2" fill="none" />
        {/* X축 */}
        <line x1={0} y1={center} x2={size} y2={center} stroke="#ddd" />
        {/* Y축 */}
        <line x1={center} y1={0} x2={center} y2={size} stroke="#ddd" />
        {/* 현재 G-force 위치 */}
        <circle cx={x} cy={y} r={8} fill="#FF6B6B" />
      </svg>

      {/* 카드 하단 텍스트 */}
      <div style={{ display: "flex", justifyContent: "space-between", marginTop: "15px" }}>
        <span>Longitudinal: {gForce.g_longitudinal.toFixed(2)}</span>
        <span>Lateral: {gForce.g_lateral.toFixed(2)}</span>
      </div>
    </div>
  );
}
