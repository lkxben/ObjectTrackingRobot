import React from 'react'
import ModeSelector from '../ModeSelector/ModeSelector'
import AutoControls from '../AutoControls/AutoControls'
import PromptControls from '../PromptControls/PromptControls'
import TrackControls from '../TrackControls/TrackControls'
import './ControlsPanel.css'

export default function ControlsPanel({
  mode,
  setMode,
  handleModeChange,
  prompt,
  setPrompt,
  targetId,
  setTargetId,
  autoTrackEnabled,
  setAutoTrackEnabled,
  handleAutoStart,
  handlePromptSubmit,
  handlePromptReset,
  handleTargetIdSubmit,
  handleTargetIdClear
}) {
  return (
    <div className="controls-root">
      <div className="controls-side">
        <ModeSelector mode={mode} onChange={handleModeChange} />
      </div>

      <div className="controls-main">
        <PromptControls
          prompt={prompt}
          setPrompt={setPrompt}
          onSubmit={handlePromptSubmit}
          onReset={handlePromptReset}
        />

        {['AUTO_LOG', 'AUTO_TRACK', 'AUTO'].includes(mode) && (
          <AutoControls
            mode={mode}
            setMode={setMode}
            prompt={prompt}
            setPrompt={setPrompt}
            handlePromptReset={handlePromptReset}
            autoTrackEnabled={autoTrackEnabled}
            setAutoTrackEnabled={setAutoTrackEnabled}
            onStart={handleAutoStart}
          />
        )}

        {mode === 'TRACK' && (
          <TrackControls
            targetId={targetId}
            setTargetId={setTargetId}
            onSubmit={handleTargetIdSubmit}
            onClear={handleTargetIdClear}
          />
        )}
      </div>
    </div>
  )
}