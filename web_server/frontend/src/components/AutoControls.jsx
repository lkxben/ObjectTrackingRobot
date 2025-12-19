import React from 'react';

function AutoControls({ prompt, autoTrackEnabled, setAutoTrackEnabled, onStart }) {
  return (
    <div className="auto-controls">
      <div className="segmented-control">
        <button
          className={!autoTrackEnabled ? 'active' : ''}
          onClick={() => setAutoTrackEnabled(false)}
        >
          Log Only
        </button>
        <button
          className={autoTrackEnabled ? 'active' : ''}
          onClick={() => setAutoTrackEnabled(true)}
        >
          Auto Track
        </button>

        <span className={`indicator ${autoTrackEnabled ? 'right' : 'left'}`} />
      </div>

      <button
        className="action-btn"
        id="start-btn"
        onClick={onStart}
        disabled={!prompt.trim()}
      >
        Start
      </button>
    </div>
  );
}

export default AutoControls;