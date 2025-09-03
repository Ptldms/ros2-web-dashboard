import logo from './logo.svg';
import './App.css';
import YawChart from './components/yaw_chart';
import GPSNavigator from './components/gps_navigator';
import GGDiagram from './components/gg_diagram';
import SpeedChart from './components/speed_chart';
import SteerChart from './components/steer_chart';

function App() {
  return (
    <div className='dashboard'>
      {/* 위쪽: Yaw (PlotJuggler) + ? + gg diagram*/}
      <div className='top-row'>
        <div className='card'>
          <h2 className="card-title">Yaw Estimation (PlotJuggler)</h2>
          <YawChart />
        </div>
        <div className='card gps-card'>
          <h2 className="card-title">YAW?GPSNavigator?</h2>
          {/*<GPSNavigator />*/}
        </div>
        <div className='card'>
          <h2 className="card-title">GG Diagram</h2>
          <GGDiagram />
        </div>
      </div>

      {/* 아래쪽: Speed + Steering */}
      <div className='bottom-row'>
        <div className='card'>
          <h2 className="card-title">Speed Comparison</h2>
          <SpeedChart />
        </div>
        <div className='card'>
          <h2 className="card-title">Steering Comparison</h2>
          <SteerChart />
        </div>
      </div>
    </div>
  );
}

export default App;
