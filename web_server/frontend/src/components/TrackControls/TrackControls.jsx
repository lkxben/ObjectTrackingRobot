import React from 'react';
import './TrackControls.css'

function TrackControls({ targetId, setTargetId, onSubmit }) {
  return (
    <div className="track-inputs">
      <input
        type="number"
        placeholder="Enter target ID"
        value={targetId}
        onChange={e => setTargetId(e.target.value)}
      />
      <button onClick={onSubmit}>Track ID</button>
    </div>
  );
}

export default TrackControls;