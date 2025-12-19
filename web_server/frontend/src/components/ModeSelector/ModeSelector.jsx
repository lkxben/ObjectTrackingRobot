import React from 'react';
import './ModeSelector.css';

function ModeSelector({ mode, onChange }) {
  const options = ['IDLE', 'TRACK', 'AUTO'];
  const sliderMode = ['AUTO_LOG', 'AUTO_TRACK'].includes(mode) ? 'AUTO' : mode;
  const activeIndex = options.indexOf(sliderMode);

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