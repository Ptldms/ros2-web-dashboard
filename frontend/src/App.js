import logo from './logo.svg';
import './App.css';
import YawChart from './components/yaw_chart';
import GForceMeter from './components/gforce_meter';
import SpeedChart from './components/speed_chart';
import SteerChart from './components/steer_chart';
import HealthStatus from './components/health_status';

function App() {
  return (
    <div className='dashboard'>
      {/* 위쪽: Yaw (PlotJuggler) + Yaw (NavBall) + G-Force Meter*/}
      <div className='top-row'>
        <div className='card'>
          <h2 className="card-title">Yaw Estimation (PlotJuggler)</h2>
          <YawChart />
        </div>
        <div className='card'>
          <h2 className="card-title">YAW Estimation (NavBall)</h2>
          {/*<YawNavBall />*/}
        </div>
        <div className='card'>
          <h2 className="card-title">G-Force Meter</h2>
          <GForceMeter />
        </div>
      </div>

      {/* 가운데: Speed + Steering */}
      <div className='middle-row'>
        <div className='card'>
          <h2 className="card-title">Speed Comparison</h2>
          <SpeedChart />
        </div>
        <div className='card'>
          <h2 className="card-title">Steering Comparison</h2>
          <SteerChart />
        </div>
      </div>
      {/* 아래쪽: Speed + Steering */}
      <div className='bottom-row'>
        <HealthStatus />
      </div>
    </div>
  );
}

export default App;
