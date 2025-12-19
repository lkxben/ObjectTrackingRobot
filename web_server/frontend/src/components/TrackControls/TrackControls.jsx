import React, { useRef } from 'react';
import './TrackControls.css'

function TrackControls({ targetId, setTargetId, onSubmit, onClear }) {
  const applied = targetId !== '';
  const inputRef = useRef(null);

  const handleButton = () => {
    if (!applied && inputRef.current.value.trim() !== '') {
      const value = inputRef.current.value;
      setTargetId(value);
      onSubmit();
    } else {
      onClear();
      setTargetId('');
      inputRef.current.value = '';
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter') handleButton();
  };

  return (
    <div className="track-inputs">
      <div className='input-wrapper'>
        <input
          type="text"
          placeholder="Track by ID"
          ref={inputRef}
          onKeyDown={handleKeyDown}
        />
        <button className="track-btn" onClick={onSubmit}>
          Track
        </button>
      </div>
    </div>
  );
}

export default TrackControls;