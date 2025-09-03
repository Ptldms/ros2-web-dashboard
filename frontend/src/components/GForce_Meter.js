import React, { useEffect, useState } from "react";

export default function GForceMeter() {
  const [gForce, setGForce] = useState({ g_longitudinal: 0, g_lateral: 0 });
  const maxG = 1.5; // 최대 G 값 (±2G)

  useEffect(() => {
    const ws = new WebSocket("ws://localhost:5000");

    ws.onmessage = (event) => {
      const data = JSON.parse(event.data);

      // g_longitudinal: 앞뒤, g_lateral: 좌우
      if (data.g_longitudinal !== undefined && data.g_lateral !== undefined) {
        setGForce({
          g_longitudinal: data.g_longitudinal,
          g_lateral: data.g_lateral,
        });
      }
    };

    return () => ws.close();
  }, []);

  const size = 220;
  const center = size / 2;
  const scale = center / (maxG * 2); // G 값 → 픽셀 변환 비율

  // 좌표 계산 (앞뒤 G = Y축, 좌우 G = X축)
  const x = center + gForce.g_lateral * scale * 2;
  const y = center + gForce.g_longitudinal * scale * 2;

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
        {/* 바깥 원 */}
        <circle cx={center} cy={center} r={center - 5} stroke="#ccc" strokeWidth="2" fill="none" />
        {/* X축 */}
        <line x1={0} y1={center} x2={size} y2={center} stroke="#ddd" />
        {/* Y축 */}
        <line x1={center} y1={0} x2={center} y2={size} stroke="#ddd" />
        {/* 현재 G-force 위치 */}
        <circle cx={x} cy={y} r={8} fill="#FF6B6B" />
      </svg>

      {/* 카드 하단 텍스트 */}
      <div style={{ display: "flex", justifyContent: "space-between", marginTop: "15px" }}>
        <span>Longitudinal: {gForce.g_longitudinal.toFixed(2)} G</span>
        <span>Lateral: {gForce.g_lateral.toFixed(2)} G</span>
      </div>
    </div>
  );
}
