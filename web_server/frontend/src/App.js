import React, { useEffect, useState, useRef, useCallback } from 'react';
import ROSLIB from 'roslib';
import './App.css';
import CameraPanel from './components/CameraPanel';
import AutoControls from './components/AutoControls';
import ManualControls from './components/ManualControls';
import PromptControls from './components/PromptControls';
import TrackControls from './components/TrackControls';
import EventConsole from './components/EventConsole';

function App() {
  const [fps, setFps] = useState(0);

  const [mode, setMode] = useState('IDLE');
  const [prompt, setPrompt] = useState('');
  const [targetId, setTargetId] = useState('');
  const [eventLogs, setEventLogs] = useState([]);
  const [filterLevel, setFilterLevel] = useState('info');
  const [autoTrackEnabled, setAutoTrackEnabled] = useState(false);
  const [streamActive, setStreamActive] = useState(false);

  const imgRef = useRef(null);
  const lastImageTimeRef = useRef(Date.now() - 10);
  const frameCountRef = useRef(0);
  const lastFpsUpdateRef = useRef(Date.now());
  const rosRef = useRef(null);
  const cameraTopicRef = useRef(null);
  const inputTopicRef = useRef(null);
  const logTopicRef = useRef(null);
  const manualTopicRef = useRef(null);
  const manualIntervalRef = useRef(null);
  const manualKeyActiveRef = useRef(null);
  const consoleRef = useRef(null);

  const severityRank = {
    debug: 1,
    info: 2,
    warning: 3,
    error: 4,
  };

  // auto scroll log
  useEffect(() => {
    if (consoleRef.current) {
      consoleRef.current.scrollTop = consoleRef.current.scrollHeight;
    }
  }, [eventLogs]);

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: 'wss://objecttrackingrobot.onrender.com/rosbridge?client=frontend' });
    rosRef.current = ros;

    inputTopicRef.current = new ROSLIB.Topic({
      ros,
      name: '/input',
      messageType: 'robot_msgs/Input',
    });
    logTopicRef.current = new ROSLIB.Topic({
      ros,
      name: '/turret/log',
      messageType: 'robot_msgs/TurretLog'
    });
    logTopicRef.current.subscribe((msg) => {
      const logEntry = {
        level: msg.level,
        message: `[${new Date(msg.stamp.sec * 1000).toLocaleTimeString()}] ${msg.message}`
      };

      setEventLogs(prev => {
        const newLogs = [...prev, logEntry].slice(-100);
        return newLogs;
      });
    });

    manualTopicRef.current = new ROSLIB.Topic({
      ros,
      name: '/motor/manual',
      messageType: 'std_msgs/Float32',
    });
    
    cameraTopicRef.current = new ROSLIB.Topic({ ros, name: '/camera/annotated/compressed', messageType: 'sensor_msgs/CompressedImage' });
    cameraTopicRef.current.subscribe((msg) => {
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
    });

    const interval = setInterval(() => {
      setStreamActive(Date.now() - lastImageTimeRef.current < 5000);
    }, 1000);

    return () => clearInterval(interval);
  }, []);

  const sendInput = ({ mode, prompt, targetId, clearPrompt, clearTarget }) => {
    const msg = new ROSLIB.Message({
      mode: mode ?? "",
      prompt: prompt ?? "",
      target_id: targetId !== undefined ? parseInt(targetId, 10) : -1,
      clear_prompt: !!clearPrompt,
      clear_target_id: !!clearTarget
    });

    inputTopicRef.current.publish(msg);
  };

  useEffect(() => {
    if (!rosRef.current) return;

    const heartbeatTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/viewer/heartbeat',
      messageType: 'std_msgs/Empty'
    });

    const interval = setInterval(() => {
      heartbeatTopic.publish(new ROSLIB.Message({}));
    }, 30000);

    return () => clearInterval(interval);
  }, []);

  const handleModeChange = (newMode) => {
    setMode(newMode);
    setPrompt('');
    setTargetId('');
    setEventLogs([]);
    if (newMode !== 'AUTO') {
      sendInput({ mode: newMode, clearPrompt: true, clearTarget: true });
    }
  };

  const handlePromptSubmit = () => {
    if (!prompt.trim()) return;
    sendInput({ prompt: prompt });
  };

  const handleClearPrompt = () => {
    setPrompt('');
    setTargetId('');
    sendInput({ clearPrompt: true, clearTarget: true });
  };

  const handleTargetIdSubmit = () => {
    if (!targetId.trim()) return;
    sendInput({ targetId: targetId });
  };

  const sendManual = (delta) => {
    if (!manualTopicRef.current) return;

    manualTopicRef.current.publish(
      new ROSLIB.Message({ data: delta })
    );
  };

  const startManual = useCallback((delta) => {
    if (!manualTopicRef.current) return;
    sendManual(delta);
    manualIntervalRef.current = setInterval(() => sendManual(delta), 50);
  }, []);

  const stopManual = () => {
    if (manualIntervalRef.current) {
      clearInterval(manualIntervalRef.current);
      manualIntervalRef.current = null;
    }
  };

  const handleAutoStart = () => {
    const modeToSend = autoTrackEnabled ? 'AUTO_TRACK' : 'AUTO_LOG';
    sendInput({ mode: modeToSend, prompt, clearPrompt: false, clearTarget: true });
  };

  useEffect(() => {
    if (!streamActive) {
      stopManual();
    }
  }, [streamActive]);

  const filteredLogs = eventLogs.filter(log => {
    return severityRank[log.level] >= severityRank[filterLevel];
  });

  useEffect(() => {
    const handleKeyDown = (e) => {
      if (mode !== 'MANUAL') return;
      if (!streamActive) return;
      if (manualKeyActiveRef.current !== null) return;

      if (['ArrowLeft', 'ArrowRight'].includes(e.key)) {
        e.preventDefault();
      }

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
      if (
        e.key === 'a' ||
        e.key === 'd' ||
        e.key === 'ArrowLeft' ||
        e.key === 'ArrowRight'
      ) {
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
  }, [mode, streamActive, startManual]);

  return (
    <div className="dashboard-container">
      <main className="dashboard-main">
        <CameraPanel
          imgRef={imgRef}
          streamActive={streamActive}
          fps={fps}
          mode={mode}
          onModeChange={handleModeChange}
        >
          {mode === 'MANUAL' && (
            <ManualControls
              enabled={streamActive}
              startManual={startManual}
              stopManual={stopManual}
            />
          )}

          {mode === 'AUTO' && (
            <AutoControls
              prompt={prompt}
              autoTrackEnabled={autoTrackEnabled}
              setAutoTrackEnabled={setAutoTrackEnabled}
              onStart={handleAutoStart}
            />
          )}

          {(mode === 'IDLE' || mode === 'TRACK' || mode === 'AUTO') && (
            <PromptControls
              prompt={prompt}
              setPrompt={setPrompt}
              onSubmit={handlePromptSubmit}
              onClear={handleClearPrompt}
            />
          )}

          {mode === 'TRACK' && (
            <TrackControls
              targetId={targetId}
              setTargetId={setTargetId}
              onSubmit={handleTargetIdSubmit}
            />
          )}
        </CameraPanel>

        <EventConsole
          logs={filteredLogs}
          filterLevel={filterLevel}
          onFilterChange={setFilterLevel}
        />
      </main>
    </div>
  );
}

export default App;