import React from 'react'
import './ManualControlPanel.css'

export default function ManualControlPad({ startManual, stopManual, mode, handleModeChange }) {
  const manualEnabled = mode === 'MANUAL'

  const toggleManual = () => {
    if (manualEnabled) {
      stopManual()
      handleModeChange('IDLE')
    } else {
      handleModeChange('MANUAL')
    }
  }

  const handleDirection = (delta) => {
    if (mode !== 'MANUAL') return
    startManual(delta)
  }

  const handleStop = () => {
    if (mode !== 'MANUAL') return
    stopManual()
  }

  return (
    <div className="manual-pad">
      <button
        className="manual-center-btn"
        onClick={toggleManual}
      >
        {manualEnabled ? 'Manual On' : 'Manual Off'}
      </button>

      <div className={`directions ${manualEnabled ? 'active' : 'disabled'}`}>
        <button
          className="up no-scale"
          onMouseDown={() => handleDirection(1)}
          onMouseUp={handleStop}
          onMouseLeave={handleStop}
        >
          ▲
        </button>
        <button
          className="down no-scale"
          onMouseDown={() => handleDirection(-1)}
          onMouseUp={handleStop}
          onMouseLeave={handleStop}
        >
          ▼
        </button>
        <button
          className="left no-scale"
          onMouseDown={() => handleDirection(-1)}
          onMouseUp={handleStop}
          onMouseLeave={handleStop}
        >
          ◀
        </button>
        <button
          className="right no-scale"
          onMouseDown={() => handleDirection(1)}
          onMouseUp={handleStop}
          onMouseLeave={handleStop}
        >
          ▶
        </button>
      </div>
    </div>
  )
}