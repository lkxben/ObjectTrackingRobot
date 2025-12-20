import React, { useState, useRef } from 'react'
import './AutoControls.css'

function AutoControls({
  mode,
  prompt,
  handlePromptReset,
  handleModeChange,
  autoTrackEnabled,
  setAutoTrackEnabled,
  onStart
}) {
  const [hovering, setHovering] = useState(false)

  const isAutoRunning = ['AUTO_LOG', 'AUTO_TRACK'].includes(mode)

  const lastAutoRef = useRef(autoTrackEnabled)

  if (!isAutoRunning) {
    lastAutoRef.current = autoTrackEnabled
  }

  return (
    <div className="auto-controls">
      <div className={`segmented-control ${isAutoRunning ? 'locked' : ''}`}>
        <button
          className={!lastAutoRef.current ? 'active' : ''}
          onClick={() => !isAutoRunning && setAutoTrackEnabled(false)}
          onMouseEnter={() => !lastAutoRef.current && setHovering(true)}
          onMouseLeave={() => setHovering(false)}
          disabled={isAutoRunning}
        >
          Log Only
        </button>

        <button
          className={lastAutoRef.current ? 'active' : ''}
          onClick={() => !isAutoRunning && setAutoTrackEnabled(true)}
          onMouseEnter={() => lastAutoRef.current && setHovering(true)}
          onMouseLeave={() => setHovering(false)}
          disabled={isAutoRunning}
        >
          Auto Track
        </button>

        <span
          className={`indicator ${hovering ? 'hover' : ''}`}
          style={{
            transform: `translateX(${lastAutoRef.current ? 100 : 0}%)`
          }}
        />
      </div>

      <button
        className="action-btn"
        disabled={!prompt.trim() && autoTrackEnabled && !isAutoRunning}
        onClick={() => {
          if (isAutoRunning) {
            handlePromptReset()
            handleModeChange('IDLE')
          } else {
            onStart()
          }
        }}
      >
        {isAutoRunning ? 'Stop' : 'Start'}
      </button>
    </div>
  )
}

export default AutoControls