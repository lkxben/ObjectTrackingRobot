import React, { useEffect, useRef, useState } from 'react';
import './EventPanel.css';

const LOG_LEVELS = ['debug', 'info', 'warning', 'error'];

function EventPanel({ logs, filterLevel, onFilterChange }) {
  const consoleRef = useRef(null);
  const [currentIndex, setCurrentIndex] = useState(LOG_LEVELS.indexOf(filterLevel));

  // Auto-scroll when logs change
  useEffect(() => {
    if (consoleRef.current) {
      consoleRef.current.scrollTop = consoleRef.current.scrollHeight;
    }
  }, [logs]);

  // Sync index when filterLevel prop changes externally
  useEffect(() => {
    setCurrentIndex(LOG_LEVELS.indexOf(filterLevel));
  }, [filterLevel]);

  const handleCycle = () => {
    const nextIndex = (currentIndex + 1) % LOG_LEVELS.length;
    setCurrentIndex(nextIndex);
    onFilterChange(LOG_LEVELS[nextIndex]);
  };

  return (
    <section className="event-console">
      <div className="event-console-header">
        <h3>Event Log</h3>
        <button className="cycle-btn" onClick={handleCycle}>
          {LOG_LEVELS[currentIndex].charAt(0).toUpperCase() + LOG_LEVELS[currentIndex].slice(1)}
        </button>
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