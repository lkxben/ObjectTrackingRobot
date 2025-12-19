import React, { useEffect, useRef } from 'react';
import './EventPanel.css'

function EventPanel({ logs, filterLevel, onFilterChange }) {
  const consoleRef = useRef(null);

  // Auto-scroll when logs change
  useEffect(() => {
    if (consoleRef.current) {
      consoleRef.current.scrollTop = consoleRef.current.scrollHeight;
    }
  }, [logs]);

  return (
    <section className="event-console">
      <div className="event-console-header">
        <h3>Event Log</h3>
        <select value={filterLevel} onChange={e => onFilterChange(e.target.value)}>
          <option value="debug">Debug</option>
          <option value="info">Info</option>
          <option value="warning">Warning</option>
          <option value="error">Error</option>
        </select>
      </div>

      <div className="log-messages" ref={consoleRef}>
        {logs.map((log, idx) => (
          <div key={idx} className={log.level}>
            {log.message}
          </div>
        ))}
      </div>
    </section>
  );
}

export default EventPanel;