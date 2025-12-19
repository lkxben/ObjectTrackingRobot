import React from 'react';
import './ModeSelector.css';

function ModeSelector({ mode, onChange }) {
  const options = ['IDLE', 'TRACK', 'AUTO'];
  const activeIndex = options.indexOf(mode);

  return (
    <div className="mode-selector">
      <div
        className="mode-slider"
        style={{ transform: `translateX(${activeIndex * 100}%)` }}
      />
      {options.map((opt, idx) => (
        <button
          key={opt}
          className={`mode-button ${activeIndex === idx ? 'active' : ''}`}
          onClick={() => onChange(opt)}
        >
          {opt}
        </button>
      ))}
    </div>
  );
}

export default ModeSelector;