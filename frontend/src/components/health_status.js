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
  { name: "Steering Angle", unit: "°"},
  { name: "Speed", unit: "m/s" },
]

const control = [
  { name: "Steer", unit: "°" },
  { name: "Speed", unit: "m/s" },
  { name: "CAN TX", unit: "Hz" },
  { name: "CAN RX", unit: "rpm" },
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
  const [alerts, setAlerts] = useState([]);

  // WebSocket 연결
  useEffect(() => {
    const host = process.env.REACT_APP_WS_HOST || "localhost";
    const port = process.env.REACT_APP_WS_PORT || "5000";
    const ws = new WebSocket(`ws://${host}:${port}`);

    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);

        if (data.type === 'monitor') {
          // Monitor 데이터 처리
          handleMonitorData(data);
        } else if (!data.type) {
          handleLegacyData(data);
        }
        
      } catch (error) {
        console.error('WebSocket message parse error:', error);
      }
    };

    const handleMonitorData = (data) => {
      const newData = {};
      if (data.sensors) {
        const sensorArray = Array.isArray(data.sensors) ? data.sensors : Object.values(data.sensors);
        sensorArray.forEach((item) => {
          newData[item.name] = item;
        });
      }
      setSensorData(newData);
      setAlerts(data.alerts || []);
    };

    const handleLegacyData = (data) => {
      const newData = {};
      if (data.sensors) {
        const sensorArray = Array.isArray(data.sensors) ? data.sensors : Object.values(data.sensors);
        sensorArray.forEach((item) => {
          newData[item.name] = item;
        });
      }
      setSensorData(newData);
      setAlerts(data.alerts || []);
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
                      <span>{data.value}</span>
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
                      <span>{data.value}</span>
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
                      <span>{data.value}</span>
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
                      <span>{data.value}</span>
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
      
      {/* Alerts */}
      <div style={styles.sectionBox}>
        <h3 style={styles.sectionTitle}>ALERTS:</h3>
        {alerts.map((a, idx) => (
          <div key={idx} style={styles.alertRow}>
            <span>{a.level === "CRITICAL" ? "🔴" : "🟡"} {a.message}</span>
          </div>
        ))}
      </div>

      {/* Performance Matrics */}
      <div style={styles.sectionBox}>
        <h3 style={styles.sectionTitle}>PERFORMANCE METRICS:</h3>
      </div>
    </div>
  );
}
