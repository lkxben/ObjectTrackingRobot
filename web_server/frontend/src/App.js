import React, { useEffect, useState, useRef } from 'react';
import ROSLIB from 'roslib';
import './App.css';

function App() {
  const [backendStatus, setBackendStatus] = useState('Connecting...');
  const [rosStatus, setRosStatus] = useState('Unknown');
  const [prompt, setPrompt] = useState('');
  const [trackId, setTrackId] = useState('');
  const [showTrackBox, setShowTrackBox] = useState(false);
  const [fps, setFps] = useState(0);
  const [trackingStatus, setTrackingStatus] = useState('');
  const imgRef = useRef(null);
  const lastImageTimeRef = useRef(Date.now());
  const localRosOnlineRef = useRef(false);
  const frameCountRef = useRef(0);
  const lastFpsUpdateRef = useRef(Date.now());

  const rosRef = useRef(null);
  const promptTopicRef = useRef(null);
  const trackIdTopicRef = useRef(null);
  const cameraTopicRef = useRef(null);

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: 'wss://objecttrackingrobot.onrender.com/rosbridge?client=frontend' });
    rosRef.current = ros;
    ros.on('connection', () => setBackendStatus('Connected'));
    ros.on('error', () => setBackendStatus('Error connecting'));
    ros.on('close', () => setBackendStatus('Disconnected'));

    promptTopicRef.current = new ROSLIB.Topic({ ros, name: '/prompt/input', messageType: 'std_msgs/String' });
    trackIdTopicRef.current = new ROSLIB.Topic({ ros, name: '/tracked/id', messageType: 'std_msgs/Int32' });
    cameraTopicRef.current = new ROSLIB.Topic({ ros, name: '/camera/annotated/compressed', messageType: 'sensor_msgs/CompressedImage' });

    cameraTopicRef.current.subscribe((msg) => {
      lastImageTimeRef.current = Date.now();
      frameCountRef.current += 1;
      const now = Date.now();
      if (now - lastFpsUpdateRef.current >= 1000) {
        setFps(frameCountRef.current);
        frameCountRef.current = 0;
        lastFpsUpdateRef.current = now;
      }
      if (!msg.data) return;
      if (!localRosOnlineRef.current) {
        localRosOnlineRef.current = true;
        setRosStatus('Online');
      }
      if (imgRef.current) imgRef.current.src = 'data:image/jpeg;base64,' + msg.data;
    });

    const interval = setInterval(() => {
      if (Date.now() - lastImageTimeRef.current > 5000) {
        localRosOnlineRef.current = false;
        setRosStatus('Offline');
      }
    }, 1000);

    return () => clearInterval(interval);
  }, []);

  const sendPrompt = (value) => {
    const trimmed = value.trim();
    if (trimmed.length > 200) return;
    promptTopicRef.current.publish(new ROSLIB.Message({ data: trimmed }));
  };

  const sendTrackId = (value) => {
    const intValue = parseInt(value, 10);
    if (isNaN(intValue)) return;
    trackIdTopicRef.current.publish(new ROSLIB.Message({ data: intValue }));
  };

  const handlePromptSubmit = () => {
    if (!prompt.trim()) return;
    sendPrompt(prompt);
    setShowTrackBox(true);
    setTrackId('');
  };

  const handleClearPrompt = () => {
    sendPrompt('');
    setPrompt('');
    setShowTrackBox(false);
    setTrackId('');
    setTrackingStatus('');
  };

  const handleTrackIdSubmit = () => {
    if (!trackId.trim()) return;
    sendTrackId(trackId);
    setTrackingStatus(`Tracking "${prompt}" (ID ${trackId})`);
  };

  const streamActive = rosStatus === 'Online' && Date.now() - lastImageTimeRef.current < 5000;

  return (
    <div className="dashboard-container">
      <main className="dashboard-main">
        <section className="camera-card">
          <div className="camera-image-wrapper">
            {streamActive ? (
              <img ref={imgRef} src="" alt="Camera stream" />
            ) : (
              <div className="placeholder-text">Waiting for stream...</div>
            )}
          </div>

          <div className="camera-status">
            <span>Backend: {backendStatus}</span>
            <span>Local ROS: {rosStatus}</span>
            <span>FPS: {fps}</span>
          </div>

          <div className="stream-controls">
            <button className="start-stream">Start Stream</button>
            <button className="stop-stream">Stop Stream</button>
          </div>
        </section>

        <section className="prompt-card">
          <div className="prompt-inputs">
            <input type="text" placeholder="Enter prompts, comma-separated" value={prompt} onChange={e => setPrompt(e.target.value)} />
            <button onClick={handlePromptSubmit}>Prompt</button>
            <button onClick={handleClearPrompt}>Clear</button>
          </div>

          {showTrackBox && (
            <div className="track-inputs">
              <input type="number" placeholder="Enter tracking ID" value={trackId} onChange={e => setTrackId(e.target.value)} />
              <button onClick={handleTrackIdSubmit}>Track ID</button>
            </div>
          )}

          {trackingStatus && <div className="tracking-status">{trackingStatus}</div>}
        </section>
      </main>
    </div>
  );
}

export default App;