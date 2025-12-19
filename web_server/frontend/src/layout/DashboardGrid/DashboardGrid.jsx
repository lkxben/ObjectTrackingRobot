import TopBar from '../TopBar/TopBar'
import './DashboardGrid.css'

export default function DashboardGrid({
  topLeft,
  topRight,
  bottomLeft,
  bottomRight
}) {
  return (
    <div className="dashboard-root">
      {/* <TopBar /> */}

      <div className="dashboard-grid">
        <div className="panel top-left">{topLeft}</div>
        <div className="panel top-right">{topRight}</div>
        <div className="panel bottom-left">{bottomLeft}</div>
        <div className="panel bottom-right">{bottomRight}</div>
      </div>
    </div>
  )
}