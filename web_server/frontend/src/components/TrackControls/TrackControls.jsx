import React, { useState, useEffect } from 'react';
import './TrackControls.css';

function TrackControls({ targetId, onSubmit, onClear }) {
  const [inputValue, setInputValue] = useState('');
  const [applied, setApplied] = useState(false);

  useEffect(() => {
    if (targetId === '') {
      setInputValue('');
      setApplied(false);
    }
  }, [targetId]);

  const handleButton = () => {
    const trimmed = inputValue.trim();

    if (!applied && trimmed !== '') {
      onSubmit(trimmed);
      setApplied(true);
    } else if (applied) {
      onClear();
      setInputValue('');
      setApplied(false);
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter') handleButton();
  };

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
  );
}

export default TrackControls;