import React, { useEffect, useState } from "react";

const perception = [
  { name: "Cam1", unit: "Hz" },
  { name: "Cam2", unit: "Hz" },
  { name: "LiDAR", unit: "Hz" },
  { name: "Fusion", unit: "cones" },
  { name: "GPS", unit: "Fix" },
  { name: "IMU", unit: "Hz" },
];

const planning = [
  { name: "Global", unit: "pts" },
  { name: "Local", unit: "Hz" },
  { name: "Speed", unit: "m/s" },
  { name: "Corridor", unit: "m" },
  { name: "Cones(L)", unit: "" },
  { name: "Cones(R)", unit: "" },
]

const control = [
  { name: "Steer", unit: "Hz" },
  { name: "RPM", unit: "rpm" },
  { name: "CAN TX", unit: "Hz" },
  { name: "CAN RX", unit: "rpm" },
  { name: "Error", unit: "°" },
  { name: "Arduino", unit: "Hz" },
]

const safety = [
  { name: "E-STOP", unit: "" },
  { name: "MODE", unit: "" },
  { name: "CAN", unit: "" },
  { name: "AEB", unit: "red" },
]

export default function HealthStatus() {
  const [sensorData, setSensorData] = useState({});

  // WebSocket 연결
  useEffect(() => {
    const host = process.env.REACT_APP_WS_HOST || "localhost";
    const port = process.env.REACT_APP_WS_PORT || "5000";
    const ws = new WebSocket(`ws://${host}:${port}`);

    ws.onmessage = (event) => {
      const data = JSON.parse(event.data); // Python에서 보낸 JSON 파싱
      const newData = {};
      data.forEach((item) => {
        newData[item.name] = item; // Cam1, Cam2 데이터 저장
      });
      setSensorData(newData);
    };

    ws.onclose = () => {
      console.log("WebSocket closed");
    };

    return () => ws.close();
  }, []);

  // 스타일 객체
  const styles = {
    container: {
      maxWidth: "800px",
      margin: "20px auto",
      padding: "16px",
      borderRadius: "8px",
      fontFamily: "monospace",
      backgroundColor: "#ffffff",
      boxShadow: "0px 4px 12px rgba(0,0,0,0.3)",
      color: "#000000",
      lineHeight: "1.5",
    },
    sectionGrid: {
      display: "grid",
      gridTemplateColumns: "repeat(3, 1fr)",
      gap: "16px",
      marginBottom: "16px",
    },
    sectionBox: {
      border: "1px solid #555",
      padding: "8px",
      borderRadius: "6px",
      backgroundColor: "#ffffff",
    },
    sectionTitle: {
      fontWeight: "bold",
      marginTop: "0px",
      marginBottom: "8px",
    },
    row: {
      display: "flex",
      justifyContent: "space-between",
      marginBottom: "2px",
    },
    safetyDiv: {
      display: "flex",
      flexDirection: "rows",
      alignItems: "center",
    },
    safetyRow: {
      display: "flex",
      gap: "16px",
    },
    alertRow: {
      marginBottom: "4px",
    },
  };

  return (
    <div style={styles.container}>
      <h2
        style={{
          ...styles.sectionTitle,
          textAlign: "center",
          fontSize: "20px",
        }}
      >
        Health Status
      </h2>

      <div style={styles.sectionGrid}>
        {/* Perception */}
        <div style={styles.sectionBox}>
          <h3 style={styles.sectionTitle}>PERCEPTION</h3>

          {perception.map((s) => {
            const data = sensorData[s.name];
            return (
              <div key={s.name} style={styles.row}>
                <span>▪ {s.name}</span>
                <div style={{ display: "flex", gap: "10px" }}>
                  {data ? (
                    <>
                      <span style={{ color: data.color }}>
                        {data.status === "GO" ? "✓" : "✗"}
                      </span>
                      <span>{data.show}</span>
                      <span>{s.unit}</span>
                    </>
                  ) : (
                    <>
                      <span style={{ color: "gray" }}>--</span>
                      <span>{s.unit}</span>
                    </>
                  )}
                </div>
              </div>
            );
          })}
        </div>

        {/* Planning */}
        <div style={styles.sectionBox}>
          <h3 style={styles.sectionTitle}>PLANNING</h3>

          {planning.map((s) => {
            const data = sensorData[s.name];
            return (
              <div key={s.name} style={styles.row}>
                <span>▪ {s.name}</span>
                <div style={{ display: "flex", gap: "10px" }}>
                  {data ? (
                    <>
                      <span style={{ color: data.color }}>
                        {data.status === "GO" ? "✓" : "✗"}
                      </span>
                      <span>{data.show}</span>
                      <span>{s.unit}</span>
                    </>
                  ) : (
                    <>
                      <span style={{ color: "gray" }}>--</span>
                      <span>{s.unit}</span>
                    </>
                  )}
                </div>
              </div>
            );
          })}
        </div>

        {/* Control */}
        <div style={styles.sectionBox}>
          <h3 style={styles.sectionTitle}>CONTROL</h3>

          {control.map((s) => {
            const data = sensorData[s.name];
            return (
              <div key={s.name} style={styles.row}>
                <span>▪ {s.name}</span>
                <div style={{ display: "flex", gap: "10px" }}>
                  {data ? (
                    <>
                      <span style={{ color: data.color }}>
                        {data.status === "GO" ? "✓" : "✗"}
                      </span>
                      <span>{data.show}</span>
                      <span>{s.unit}</span>
                    </>
                  ) : (
                    <>
                      <span style={{ color: "gray" }}>--</span>
                      <span>{s.unit}</span>
                    </>
                  )}
                </div>
              </div>
            );
          })}
        </div>
      </div>

      {/* Safety Systems */}
      <div style={styles.sectionBox}>
        <h3 style={styles.sectionTitle}>SAFETY SYSTEMS</h3>
        <div style={styles.safetyRow}>
          {safety.map((s) => {
            const data = sensorData[s.name];
            return (
              <div key={s.name} style={styles.safetyDiv}>
                <span style={{ marginRight: "12px"}}>▪ {s.name}:</span>
                <div style={styles.safetyRow}>
                  {data ? (
                    <>
                      <span style={{ color: data.color }}>
                        {data.status === "GO" ? "✓" : "✗"}
                      </span>
                      <span>{data.show}</span>
                      <span>{s.unit}</span>
                    </>
                  ) : (
                    <>
                      <span style={{ color: "gray" }}>--</span>
                      <span>{s.unit}</span>
                    </>
                  )}
                </div>
              </div>
            );
          })}
        </div>
      </div>
      
      {/* Critical Alerts */}
      <div style={styles.sectionBox}>
        <h3 style={styles.sectionTitle}>CRITICAL ALERTS:</h3>
      </div>

      {/* Performance Matrics */}
      <div style={styles.sectionBox}>
        <h3 style={styles.sectionTitle}>PERFORMANCE METRICS:</h3>
      </div>
    </div>
  );
}
