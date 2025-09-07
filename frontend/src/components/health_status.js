import React from "react";

const healthData = {
  perception: [
    { name: "Cam1", status: true, value: "23.1Hz" },
    { name: "Cam2", status: false, value: "0Hz" },
    { name: "LiDAR", status: true, value: "10.2Hz" },
    { name: "Fusion", status: true, value: "8 cones" },
    { name: "GPS", status: true, value: "RTK Fix" },
    { name: "IMU", status: true, value: "99.8Hz" },
  ],
  planning: [
    { name: "Global", status: true, value: "523pts" },
    { name: "Local", status: true, value: "22Hz" },
    { name: "Speed", status: true, value: "5.8m/s" },
    { name: "Corridor", status: true, value: "4.2m" },
    { name: "Cones L:6 R:5", status: true, value: "" },
    { name: "FSM State", status: null, value: "[0]" },
  ],
  control: [
    { name: "Steer", status: true, value: "52Hz" },
    { name: "RPM", status: true, value: "850" },
    { name: "CAN TX", status: true, value: "20Hz" },
    { name: "CAN RX", status: true, value: "856rpm" },
    { name: "Error", status: true, value: "0.8°" },
    { name: "Arduino", status: true, value: "99.8Hz" },
  ],
  safety: [
    { name: "E-STOP", status: false, value: "OFF" },
    { name: "MODE", status: null, value: "AUTO" },
    { name: "CAN", status: true, value: "OK" },
    { name: "AEB", status: null, value: "2 red" },
  ],
  alerts: [
    "⚠ GPS covariance increasing (0.00018 → 0.00025)",
    "⚠ Left cone detection sparse (only 3 cones in view)",
  ],
  performance: "CPU: 67% | RAM: 4.2GB/8GB | Latency: 42ms | FPS: 48.2",
};

// Status 표시 컴포넌트
const StatusIndicator = ({ status }) => {
  const style = { marginRight: "4px" };
  if (status === null) return <span style={{ ...style, color: "gray" }}>■</span>;
  return status ? <span style={{ ...style, color: "lime" }}>✓</span> : <span style={{ ...style, color: "red" }}>✗</span>;
};

export default function HealthStatus() {
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
    safetyRow: {
      display: "flex",
      flexWrap: "wrap",
      gap: "12px",
    },
    alertRow: {
      marginBottom: "4px",
    },
  };

  return (
    <div style={styles.container}>
      <h2 style={{ ...styles.sectionTitle, textAlign: "center", fontSize: "20px" }}>Autonomous Vehicle Monitor</h2>

      <div style={styles.sectionGrid}>
        {/* Perception */}
        <div style={styles.sectionBox}>
          <h3 style={styles.sectionTitle}>PERCEPTION [GO]</h3>
          {healthData.perception.map((item, idx) => (
            <div key={idx} style={styles.row}>
              <span>▪ {item.name}</span>
              <span>
                <StatusIndicator status={item.status} /> {item.value}
              </span>
            </div>
          ))}
        </div>

        {/* Planning */}
        <div style={styles.sectionBox}>
          <h3 style={styles.sectionTitle}>PLANNING [GO]</h3>
          {healthData.planning.map((item, idx) => (
            <div key={idx} style={styles.row}>
              <span>▪ {item.name}</span>
              <span>
                <StatusIndicator status={item.status} /> {item.value}
              </span>
            </div>
          ))}
        </div>

        {/* Control */}
        <div style={styles.sectionBox}>
          <h3 style={styles.sectionTitle}>CONTROL [GO]</h3>
          {healthData.control.map((item, idx) => (
            <div key={idx} style={styles.row}>
              <span>▪ {item.name}</span>
              <span>
                <StatusIndicator status={item.status} /> {item.value}
              </span>
            </div>
          ))}
        </div>
      </div>

      {/* Safety */}
      <div style={styles.sectionBox}>
        <h3 style={styles.sectionTitle}>SAFETY SYSTEMS [ALL CLEAR]</h3>
        <div style={styles.safetyRow}>
          {healthData.safety.map((item, idx) => (
            <div key={idx}>
              ▪ {item.name}: <StatusIndicator status={item.status} /> {item.value}
            </div>
          ))}
        </div>
      </div>

      {/* Alerts */}
      <div style={styles.sectionBox}>
        <h3 style={styles.sectionTitle}>CRITICAL ALERTS:</h3>
        {healthData.alerts.map((alert, idx) => (
          <div key={idx} style={styles.alertRow}>{alert}</div>
        ))}
      </div>

      {/* Performance */}
      <div style={styles.sectionBox}>
        <h3 style={styles.sectionTitle}>PERFORMANCE METRICS:</h3>
        <div>{healthData.performance}</div>
      </div>
    </div>
  );
}
