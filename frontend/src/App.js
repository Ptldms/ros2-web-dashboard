import logo from './logo.svg';
import './App.css';
import YawChart from './components/yaw_chart';
import GPSNavigator from './components/gps_navigator';
import GForce_Meter from './components/GForce_Meter';
import SpeedChart from './components/speed_chart';
import SteerChart from './components/steer_chart';

function App() {
  return (
    <div className='dashboard'>
      {/* 위쪽: Yaw (PlotJuggler) + ? + G-Force Meter*/}
      <div className='top-row'>
        <div className='card'>
          <h2 className="card-title">Yaw Estimation (PlotJuggler)</h2>
          <YawChart />
        </div>
        <div className='card'>
          <h2 className="card-title">YAW NavBall</h2>
          {/*<GPSNavigator />*/}
        </div>
        <div className='card'>
          <h2 className="card-title">G-Force Meter</h2>
          <GForce_Meter />
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
