import React, { useState } from 'react';
import './AutoControls.css'

function AutoControls({ mode, setMode, prompt, setPrompt, handlePromptReset, autoTrackEnabled, setAutoTrackEnabled, onStart }) {
  const [hovering, setHovering] = useState(false);

  return (
    <div className="auto-controls">
      <div className="segmented-control">
        <button
          className={!autoTrackEnabled ? 'active' : ''}
          onClick={() => setAutoTrackEnabled(false)}
          onMouseEnter={() => !autoTrackEnabled && setHovering(true)}
          onMouseLeave={() => setHovering(false)}
        >
          Log Only
        </button>
        <button
          className={autoTrackEnabled ? 'active' : ''}
          onClick={() => setAutoTrackEnabled(true)}
          onMouseEnter={() => autoTrackEnabled && setHovering(true)}
          onMouseLeave={() => setHovering(false)}
        >
          Auto Track
        </button>

        <span 
          className={`indicator ${hovering ? 'hover' : ''}`}
          style={{ transform: `translateX(${(autoTrackEnabled ? 100 : 0)}%)` }}
        />
      </div>

      <button
        className="action-btn"
        // disabled={!prompt.trim()}
        onClick={() => {
          if (mode === 'AUTO_LOG' || mode === 'AUTO_TRACK') {
            setPrompt('');
            handlePromptReset();
            setMode('IDLE');
          } else {
            onStart();
          }
        }}
      >
        {['AUTO_LOG', 'AUTO_TRACK'].includes(mode) ? 'Stop' : 'Start'}
      </button>
    </div>
  );
}

export default AutoControls;