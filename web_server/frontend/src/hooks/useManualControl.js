import { useRef, useEffect, useCallback } from 'react';

export default function useManualControl(manualTopicRef, streamActive, mode) {
  const manualIntervalRef = useRef(null);
  const manualKeyActiveRef = useRef(null);

  const sendManual = useCallback((delta) => {
    if (!manualTopicRef.current) return;
    manualTopicRef.current.publish({ data: delta });
  }, [manualTopicRef]);

  const startManual = useCallback((delta) => {
    sendManual(delta);
    manualIntervalRef.current = setInterval(() => sendManual(delta), 50);
  }, [sendManual]);

  const stopManual = useCallback(() => {
    if (manualIntervalRef.current) {
      clearInterval(manualIntervalRef.current);
      manualIntervalRef.current = null;
    }
  }, []);

  useEffect(() => {
    const handleKeyDown = (e) => {
      if (mode !== 'MANUAL' || !streamActive || manualKeyActiveRef.current !== null) return;

      if (['ArrowLeft', 'ArrowRight'].includes(e.key)) e.preventDefault();

      if (e.key === 'a' || e.key === 'ArrowLeft') {
        manualKeyActiveRef.current = -1;
        startManual(-1);
      }

      if (e.key === 'd' || e.key === 'ArrowRight') {
        manualKeyActiveRef.current = 1;
        startManual(1);
      }
    };

    const handleKeyUp = (e) => {
      if (['a', 'd', 'ArrowLeft', 'ArrowRight'].includes(e.key)) {
        manualKeyActiveRef.current = null;
        stopManual();
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [mode, streamActive, startManual, stopManual]);

  useEffect(() => {
    if (!streamActive) stopManual();
  }, [streamActive, stopManual]);

  return { startManual, stopManual };
}