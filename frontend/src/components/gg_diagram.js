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

export default function GGDiagram() {
  const [data, setData] = useState([]);

  useEffect(() => {
    const ws = new WebSocket("ws://localhost:5000");

    return () => ws.close();
  }, []);

  return (
    <div>
    </div>
  );
}
