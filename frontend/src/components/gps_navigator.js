import { useEffect, useState } from "react";
import { MapContainer, TileLayer, Marker, Polyline, useMap } from "react-leaflet";
import L from "leaflet";
import car_icon from '../assets/car_icon.png';

const createCarIcon = (yaw) =>
    L.divIcon({
        className: "",
        html: `<div style="
            transform:rotate(${yaw}rad);
            width: 40px;
            height: 40px;
            background: url(${car_icon}) no-repeat center center;
            background-size: contain;
            "></div>`,
    });

export default function GPSNavigator() {
    /*
    konkuk_250721 : 37.542109, 127.078148
    administrator_250721 : 37.543116, 127.076076
    */

    const [lat, setLat] = useState(37.543116);
    const [lon, setLon] = useState(127.076076);
    const [yaw, setYaw] = useState(0);
    const [cov, setCov] = useState(9999);
    const [path, setPath] = useState([]);
    const [map, setMap] = useState(null);
    const covThreshold = 0.01;

    // yaw WebSocket 연결
    useEffect(() => {
        const host = process.env.REACT_APP_WS_HOST || "localhost";
        const port = process.env.REACT_APP_WS_PORT || "5000";
        const ws = new WebSocket(`ws://${host}:${port}`);

        yawSocket.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                if (data.raw !== undefined) {
                    setYaw(data.raw);
                }
            } catch (e) {
                console.error("Yaw parse error", e);
            }
        }
        return () => yawSocket.close();
    }, []);

    // gps WebSocket 연결
    useEffect(() => {
        const host = process.env.REACT_APP_WS_HOST || "localhost";
        const port = process.env.REACT_APP_WS_PORT || "5000";
        const ws = new WebSocket(`ws://${host}:${port}`);
        
        gpsSocket.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                if (data.raw !== undefined) {
                    setLat(data.lat);
                    setLon(data.lon);
                    setCov(data.cov);

                    if (data.cov > covThreshold) {
                        setPath((prev) => [...prev, [data.lat, data.lon]]);
                    }
                }
            } catch (e) {
                console.error("GPS parse error", e);
            }
        };
        return () => gpsSocket.close();
    }, []);

    const recenterMap = () => {
        if (map) {
            map.setView([lat, lon], map.getZoom(), { animate: true });
        }
    };

    return (
        <div className="gps-navigator" style={{ position: "relative", height: "100%", width: "100%" }}>
            <MapContainer
                center={[lat, lon]}
                zoom={13}
                style={{ height: "100%", width: "100%" }}
                whenCreated={setMap}
            >
                <TileLayer url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" />

                {/* 차량 icon */}
                {cov < covThreshold && (
                    <Marker position={[lat, lon]} icon={createCarIcon(yaw)} />
                )}

                {/* 주행 궤적 */}
                {path.length > 1 && <Polyline positions={path} color="blue" />}
            </MapContainer>

            {/* 내 위치로 돌아가기 버튼 */}
            <button
                onClick={recenterMap}
                style={{
                    position: "absolute",
                    top: "10px",
                    right: "10px",
                    padding: "10px",
                    borderRadius: "50%",
                    border: "none",
                    background: "white",
                    boxShadow: "0 2px 6px rgba(0,0,0,0.3)",
                    cursor: "pointer",
                }}
            >
                📍
            </button>
        </div>
    );
}