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
    const ros = new ROSLIB.Ros({
      url: 'wss://handguesturerobot.onrender.com/rosbridge?client=frontend'
    });

    rosRef.current = ros;
    ros.on('connection', () => setBackendStatus('Connected'));
    ros.on('error', () => setBackendStatus('Error connecting'));
    ros.on('close', () => setBackendStatus('Disconnected'));

    promptTopicRef.current = new ROSLIB.Topic({
      ros: ros,
      name: '/prompt/input',
      messageType: 'std_msgs/String'
    });

    trackIdTopicRef.current = new ROSLIB.Topic({
      ros: ros,
      name: '/tracked/id',
      messageType: 'std_msgs/Int32'
    });

    cameraTopicRef.current = new ROSLIB.Topic({
      ros: ros,
      name: '/camera/annotated/compressed',
      messageType: 'sensor_msgs/CompressedImage'
    });

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

      if (imgRef.current) {
        imgRef.current.src = 'data:image/jpeg;base64,' + msg.data;
      }
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
    setPrompt('');
    sendPrompt(prompt);
    setShowTrackBox(false);
    setTrackId('');
    setTrackingStatus('');
  };

  const handleTrackIdSubmit = () => {
    if (!trackId.trim()) return;
    sendTrackId(trackId);
    setTrackingStatus(`Tracking "${prompt}" (ID ${trackId})`);
  };

  return (
    <div style={{ textAlign: 'center', fontFamily: 'sans-serif', background: '#111', color: '#eee', minHeight: '100vh', paddingTop: '20px' }}>
      <h1>Robot Camera</h1>
      <div>
        <img ref={imgRef} src="" alt="Waiting for stream..." style={{ width: '80vw', maxWidth: '100%', height: 'auto', border: '2px solid #555', borderRadius: '6px', marginTop: '20px' }} />
      </div>
      <div id="backendStatus" style={{ marginTop: '10px', fontSize: '1.1em' }}>Backend: {backendStatus}</div>
      <div id="rosStatus" style={{ marginTop: '5px', fontSize: '1.1em' }}>Local ROS: {rosStatus}</div>
      <div id="fps" style={{ marginTop: '5px', fontSize: '1.1em' }}>FPS: {fps}</div>

      <div style={{ marginTop: '20px' }}>
        <input
          type="text"
          placeholder="Enter prompts, comma-separated"
          value={prompt}
          onChange={e => setPrompt(e.target.value)}
          style={{ width: '300px' }}
        />
        <button onClick={handlePromptSubmit} style={{ marginLeft: '10px' }}>Prompt</button>
        <button onClick={handleClearPrompt} style={{ marginLeft: '10px' }}>Clear</button>
      </div>

      {showTrackBox && (
        <div style={{ marginTop: '10px' }}>
          <input
            type="number"
            placeholder="Enter tracking ID"
            value={trackId}
            onChange={e => setTrackId(e.target.value)}
            style={{ width: '150px' }}
          />
          <button onClick={handleTrackIdSubmit} style={{ marginLeft: '10px' }}>Track ID</button>
        </div>
      )}

      {trackingStatus && (
        <div style={{ marginTop: '10px', fontSize: '1.1em', color: '#4fd1c5' }}>
          {trackingStatus}
        </div>
      )}
    </div>
  );
}

export default App;