import React from 'react';

function ModeSelector({ mode, onChange }) {
  return (
    <div className="mode-card">
      <select value={mode} onChange={e => onChange(e.target.value)}>
        <option value="IDLE">IDLE</option>
        <option value="MANUAL">MANUAL</option>
        <option value="TRACK">TRACK</option>
        <option value="AUTO">AUTO</option>
      </select>
    </div>
  );
}

export default ModeSelector;