import React, { useRef } from 'react'
import './ModeSelector.css'

function ModeSelector({ mode, onChange }) {
  const options = ['IDLE', 'TRACK', 'AUTO']

  const isManual = mode === 'MANUAL'
  const isAutoSubMode = ['AUTO_LOG', 'AUTO_TRACK'].includes(mode)
  const isLocked = isManual || isAutoSubMode

  const resolveMode = m => (isAutoSubMode ? 'AUTO' : m)

  const lastIndexRef = useRef(0)

  if (!isManual) {
    const idx = options.indexOf(resolveMode(mode))
    if (idx !== -1) lastIndexRef.current = idx
  }

  return (
    <div
      className={`mode-selector vertical
        ${isLocked ? 'locked' : ''}
        ${isManual ? 'manual' : ''}`}
    >
      <span
        className="mode-slider"
        style={{
          transform: `translateY(${lastIndexRef.current * 100}%)`
        }}
      />
      {options.map(opt => (
        <button
          key={opt}
          className={`mode-button ${
            !isManual && resolveMode(mode) === opt ? 'active' : ''
          }`}
          onClick={() => !isLocked && onChange(opt)}
          disabled={isLocked}
        >
          {opt}
        </button>
      ))}
    </div>
  )
}

export default ModeSelector