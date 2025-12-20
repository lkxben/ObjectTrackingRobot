import React, { useState, useEffect } from 'react'
import './TrackControls.css'

function TrackControls({ targetId, setTargetId, onSubmit, onClear }) {
  const [inputValue, setInputValue] = useState('')

  // Keep input in sync with external reset
  useEffect(() => {
    if (targetId === '') setInputValue('')
  }, [targetId])

  const applied = targetId !== ''

  const handleButton = () => {
    const trimmed = inputValue.trim()
    if (!applied && trimmed !== '') {
      setTargetId(trimmed)
      onSubmit()
    } else {
      onClear()
      setTargetId('')
      setInputValue('')
    }
  }

  const handleKeyDown = (e) => {
    if (e.key === 'Enter') handleButton()
  }

  return (
    <div className="track-inputs">
      <div className="input-wrapper">
        <input
          type="text"
          placeholder="Track by ID"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyDown={handleKeyDown}
        />
        <button className="track-btn" onClick={handleButton}>
          {applied ? 'Clear' : 'Track'}
        </button>
      </div>
    </div>
  )
}

export default TrackControls