import logo from './logo.svg';
import './App.css';
import YawChart from './components/yaw_chart';
import YawNavBall from './components/yaw_navball';
import GGDiagram from './components/gg_diagram';

function App() {
  return (
    <div className='dashboard'>
      {/* 위쪽: Yaw (PlotJuggler + NavBall)*/}
      <div className='top-row'>
        <div className='card'>
          <h2 className="card-title">Yaw Estimation (PlotJuggler)</h2>
          <YawChart />
        </div>
        <div className='card'>
          <h2 className="card-title">Yaw Estimation (NavBall)</h2>
          <YawNavBall />
        </div>
      </div>

      {/* 아래쪽: gg diagram */}
      <div className='bottom-row'>
        <div className='card'>
          <h2 className="card-title">GG Diagram</h2>
          <GGDiagram />
        </div>
      </div>
    </div>
  );
}

export default App;
