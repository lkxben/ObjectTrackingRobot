import React, { useEffect, useState, useRef } from 'react';
import ROSLIB from 'roslib';
import './App.css';

function App() {
  const [backendStatus, setBackendStatus] = useState('Connecting...');
  const [rosStatus, setRosStatus] = useState('Unknown');
  const [fps, setFps] = useState(0);
  const [trackingStatus, setTrackingStatus] = useState('');

  const [mode, setMode] = useState('IDLE');
  const [prompt, setPrompt] = useState('');
  const [targetId, setTargetId] = useState('');

  const imgRef = useRef(null);
  const lastImageTimeRef = useRef(Date.now());
  const localRosOnlineRef = useRef(false);
  const frameCountRef = useRef(0);
  const lastFpsUpdateRef = useRef(Date.now());
  const rosRef = useRef(null);
  const cameraTopicRef = useRef(null);
  const inputTopicRef = useRef(null);

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: 'wss://objecttrackingrobot.onrender.com/rosbridge?client=frontend' });
    rosRef.current = ros;
    ros.on('connection', () => setBackendStatus('Connected'));
    ros.on('error', () => setBackendStatus('Error connecting'));
    ros.on('close', () => setBackendStatus('Disconnected'));

    inputTopicRef.current = new ROSLIB.Topic({
      ros,
      name: '/input',
      messageType: 'robot_msgs/Input',
    });
    
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

  const sendInput = ({ mode, prompt, target_id }) => {
    const msg = {
      mode: mode ?? "",
      prompt: prompt ?? "__NONE__",
      target_id: target_id !== undefined ? parseInt(target_id, 10) : -1
    };
    inputTopicRef.current.publish(new ROSLIB.Message(msg));
  }

  const handleModeChange = (e) => {
    const newMode = e.target.value;
    setMode(newMode);

    setPrompt('');
    setTargetId('');
    setTrackingStatus('');

    sendInput({ mode: newMode });
  };

  const handlePromptSubmit = () => {
    if (!prompt.trim()) return;
    sendInput({ prompt: prompt });
  };

  const handleClearPrompt = () => {
    setPrompt('');
    setTargetId('');
    setTrackingStatus('');
    sendInput({ prompt: "__EMPTY__", target_id: -999 });
  };

  const handleTargetIdSubmit = () => {
    if (!targetId.trim()) return;
    sendInput({ target_id: targetId });
    setTrackingStatus(`Tracking "${prompt}" (ID ${targetId})`);
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

        <section className="mode-card">
          <select value={mode} onChange={handleModeChange}>
            <option value="IDLE">IDLE</option>
            <option value="MANUAL">MANUAL</option>
            <option value="TRACK">TRACK</option>
            <option value="AUTO">AUTO</option>
          </select>
        </section>

        <section className="prompt-card">
          {(mode === 'IDLE' || mode === 'TRACK' || mode === 'AUTO') && (
            <div className="prompt-inputs">
              <input type="text" placeholder="Enter prompts, comma-separated" value={prompt} onChange={e => setPrompt(e.target.value)} />
              <button onClick={handlePromptSubmit}>Prompt</button>
              <button onClick={handleClearPrompt}>Clear</button>
            </div>
          )}

          {mode === 'TRACK' && (
            <div className="track-inputs">
              <input type="number" placeholder="Enter target ID" value={targetId} onChange={e => setTargetId(e.target.value)} />
              <button onClick={handleTargetIdSubmit}>Track ID</button>
            </div>
          )}

          {trackingStatus && <div className="tracking-status">{trackingStatus}</div>}
        </section>
      </main>
    </div>
  );
}

export default App;