import { useState, useEffect, useCallback } from 'react';

export default function useEventLogs(logTopicRef, severityRank, filterLevel) {
  const [eventLogs, setEventLogs] = useState([]);

  useEffect(() => {
    if (!logTopicRef.current) return;

    const subscriber = (msg) => {
      const logEntry = {
        level: msg.level,
        message: `[${new Date(msg.stamp.sec * 1000).toLocaleTimeString()}] ${msg.message}`,
      };

      setEventLogs(prev => [...prev, logEntry].slice(-100));
    };

    logTopicRef.current.subscribe(subscriber);

    return () => logTopicRef.current.unsubscribe(subscriber);
  }, [logTopicRef]);

  const clearLogs = useCallback(() => {
    setEventLogs([]);
  }, []);

  const filteredLogs = eventLogs.filter(
    log => severityRank[log.level] >= severityRank[filterLevel]
  );

  return { filteredLogs, clearLogs };
}