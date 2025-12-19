import React from 'react';
import './ModeSelector.css';

function ModeSelector({ mode, onChange }) {
  const options = ['IDLE', 'TRACK', 'AUTO'];
  const activeIndex = options.indexOf(mode);

  return (
    <div className="mode-selector vertical">
      <span
        className="mode-slider"
        style={{ transform: `translateY(${activeIndex * 100}%)` }}
      />
      {options.map(opt => (
        <button
          key={opt}
          className={`mode-button ${mode === opt ? 'active' : ''}`}
          onClick={() => onChange(opt)}
        >
          {opt}
        </button>
      ))}
    </div>
  );
}

export default ModeSelector;