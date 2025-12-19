import React, { useState, useRef } from 'react';
import './App.css';
import CameraPanel from './components/CameraPanel';
import AutoControls from './components/AutoControls';
import ManualControls from './components/ManualControls';
import PromptControls from './components/PromptControls';
import TrackControls from './components/TrackControls';
import EventConsole from './components/EventConsole';

import useRosConnection from './hooks/useRosConnection';
import useManualControl from './hooks/useManualControl';
import useCameraStream from './hooks/useCameraStream';
import useEventLogs from './hooks/useEventLogs';
import useInputPublisher from './hooks/useInputPublisher';

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
    setPrompt('');
    setTargetId('');
    clearLogs();
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

  const handleAutoStart = () => {
    const modeToSend = autoTrackEnabled ? 'AUTO_TRACK' : 'AUTO_LOG';
    sendInput({ mode: modeToSend, prompt, clearPrompt: false, clearTarget: true });
  };

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