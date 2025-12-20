import React, { useState, useRef } from 'react';
import './global.css'
import CameraPanel from './components/CameraPanel/CameraPanel';
import ControlsPanel from './components/ControlsPanel/ControlsPanel'
import ManualControlPanel from './components/ManualControlPanel/ManualControlPanel';
import EventPanel from './components/EventPanel/EventPanel';

import useRosConnection from './hooks/useRosConnection';
import useManualControl from './hooks/useManualControl';
import useCameraStream from './hooks/useCameraStream';
import useEventLogs from './hooks/useEventLogs';
import useInputPublisher from './hooks/useInputPublisher';

import DashboardGrid from './layout/DashboardGrid/DashboardGrid';

function App() {
  const [mode, setMode] = useState('IDLE');
  const [prompt, setPrompt] = useState('');
  const [targetId, setTargetId] = useState('');
  const [filterLevel, setFilterLevel] = useState('info');
  const [autoTrackEnabled, setAutoTrackEnabled] = useState(false);

  const imgRef = useRef(null);

  const severityRank = {
    debug: 1,
    info: 2,
    warning: 3,
    error: 4,
  };

  const { rosRef, cameraTopicRef, inputTopicRef, logTopicRef, manualTopicRef } = useRosConnection();
  const { fps, streamActive } = useCameraStream(cameraTopicRef, imgRef);
  const { filteredLogs, clearLogs } = useEventLogs(logTopicRef, severityRank, filterLevel);
  const { startManual, stopManual } = useManualControl(manualTopicRef, streamActive, mode);
  const { sendInput } = useInputPublisher(inputTopicRef);

  const handleModeChange = (newMode) => {
    setMode(newMode);
    setTargetId('');
    if (newMode !== 'AUTO') {
      sendInput({ mode: newMode, clearTarget: true });
    }
  };

  const handlePromptSubmit = (promptValue) => {
    if (!promptValue.trim()) return;
    setPrompt(promptValue);
    sendInput({ prompt: promptValue });
  };

  const handlePromptReset = () => {
    setPrompt('');
    setTargetId('');
    sendInput({ clearPrompt: true, clearTarget: true });
  };

  const handleTargetIdSubmit = (targetValue) => {
    const value = Number(targetValue.trim())
    if (!Number.isInteger(value)) return;
    setTargetId(value);
    sendInput({ targetId: value });
  }

  const handleTargetIdClear = () => {
    setTargetId('');
    sendInput({ clearTarget: true });
  }

  const handleAutoStart = () => {
    const modeToSend = autoTrackEnabled ? 'AUTO_TRACK' : 'AUTO_LOG';
    setMode(modeToSend);
    sendInput({ mode: modeToSend, prompt, clearPrompt: false, clearTarget: true });
  };

  return (
    <DashboardGrid
      topLeft={
        <CameraPanel
          imgRef={imgRef}
          streamActive={streamActive}
          fps={fps}
        />
      }

      bottomLeft={
        <ControlsPanel
          mode={mode}
          handleModeChange={handleModeChange}
          prompt={prompt}
          targetId={targetId}
          setTargetId={setTargetId}
          autoTrackEnabled={autoTrackEnabled}
          setAutoTrackEnabled={setAutoTrackEnabled}
          handleAutoStart={handleAutoStart}
          handlePromptSubmit={handlePromptSubmit}
          handlePromptReset={handlePromptReset}
          handleTargetIdSubmit={handleTargetIdSubmit}
          handleTargetIdClear={handleTargetIdClear}
        />
      }

      topRight={
        <EventPanel
          logs={filteredLogs}
          filterLevel={filterLevel}
          onFilterChange={setFilterLevel}
        />
      }

      bottomRight={
        <ManualControlPanel
          startManual={startManual}
          stopManual={stopManual}
          mode={mode}
          handleModeChange={handleModeChange}
        />
      }
    />
  )
}

export default App;