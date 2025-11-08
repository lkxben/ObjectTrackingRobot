import React, { useEffect, useState, useRef } from 'react';
import ROSLIB from 'roslib';
import './App.css';

function App() {
  const [backendStatus, setBackendStatus] = useState('Connecting...');
  const [rosStatus, setRosStatus] = useState('Unknown');
  const [prompt, setPrompt] = useState('');
  const imgRef = useRef(null);
  const lastImageTimeRef = useRef(Date.now());
  const localRosOnlineRef = useRef(false);

  useEffect(() => {
    const ros = new ROSLIB.Ros({
      url: 'wss://handguesturerobot.onrender.com/rosbridge?client=frontend'
    });

    ros.on('connection', () => setBackendStatus('Connected'));
    ros.on('error', () => setBackendStatus('Error connecting'));
    ros.on('close', () => setBackendStatus('Disconnected'));

    const cameraTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/camera/annotated/compressed',
      messageType: 'sensor_msgs/CompressedImage'
    });

    cameraTopic.subscribe((msg) => {
      lastImageTimeRef.current = Date.now();
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

  const handleSendPrompt = () => {
    const trimmed = prompt.trim();
    if (!trimmed) return;
    if (trimmed.length > 200) return alert('Prompt too long (max 200 chars)');

    const ros = new ROSLIB.Ros({
      url: 'wss://handguesturerobot.onrender.com/rosbridge?client=frontend'
    });

    const promptTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/prompt/input',
      messageType: 'std_msgs/String'
    });

    const msg = new ROSLIB.Message({ data: trimmed });
    promptTopic.publish(msg);
    setPrompt('');
  };

  return (
    <div style={{ textAlign: 'center', fontFamily: 'sans-serif', background: '#111', color: '#eee', minHeight: '100vh', paddingTop: '20px' }}>
      <h1>Robot Camera</h1>
      <div>
        <img ref={imgRef} src="" alt="Waiting for stream..." style={{ width: '80vw', maxWidth: '100%', height: 'auto', border: '2px solid #555', borderRadius: '6px', marginTop: '20px' }} />
      </div>
      <div id="backendStatus" style={{ marginTop: '10px', fontSize: '1.1em' }}>Backend: {backendStatus}</div>
      <div id="rosStatus" style={{ marginTop: '5px', fontSize: '1.1em' }}>Local ROS: {rosStatus}</div>

      <div style={{ marginTop: '20px' }}>
        <input
          type="text"
          placeholder="Enter prompts, comma-separated"
          value={prompt}
          onChange={e => setPrompt(e.target.value)}
          style={{ width: '300px' }}
        />
        <button onClick={handleSendPrompt} style={{ marginLeft: '10px' }}>Send Prompt</button>
      </div>
    </div>
  );
}

export default App;