import { useState, useEffect, useRef } from 'react';

export default function useCameraStream(cameraTopicRef, imgRef) {
  const [fps, setFps] = useState(0);
  const [streamActive, setStreamActive] = useState(false);
  const lastImageTimeRef = useRef(0);
  const frameCountRef = useRef(0);
  const lastFpsUpdateRef = useRef(Date.now());

  useEffect(() => {
    if (!cameraTopicRef.current) return;

    const subscriber = (msg) => {
      if (!msg.data) return;
      lastImageTimeRef.current = Date.now();
      frameCountRef.current += 1;

      const now = Date.now();
      if (now - lastFpsUpdateRef.current >= 1000) {
        setFps(frameCountRef.current);
        frameCountRef.current = 0;
        lastFpsUpdateRef.current = now;
      }

      if (imgRef.current) imgRef.current.src = 'data:image/jpeg;base64,' + msg.data;
    };

    cameraTopicRef.current.subscribe(subscriber);

    const interval = setInterval(() => {
      setStreamActive(Date.now() - lastImageTimeRef.current < 5000);
    }, 1000);

    return () => {
      cameraTopicRef.current.unsubscribe(subscriber);
      clearInterval(interval);
    };
  }, [cameraTopicRef, imgRef]);

  return { fps, streamActive };
}