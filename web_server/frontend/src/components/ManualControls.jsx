import React from 'react';

function ManualControls({ enabled, startManual, stopManual }) {
  return (
    <div className="manual-controls">
      <button
        disabled={!enabled}
        onMouseDown={() => startManual(-1)}
        onMouseUp={stopManual}
        onMouseLeave={stopManual}
      >
        ◀
      </button>

      <button
        disabled={!enabled}
        onMouseDown={() => startManual(1)}
        onMouseUp={stopManual}
        onMouseLeave={stopManual}
      >
        ▶
      </button>
    </div>
  );
}

export default ManualControls;