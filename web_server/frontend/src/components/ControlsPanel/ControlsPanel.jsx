import React from 'react'
import ModeSelector from '../ModeSelector/ModeSelector'
import AutoControls from '../AutoControls/AutoControls'
import PromptControls from '../PromptControls/PromptControls'
import TrackControls from '../TrackControls/TrackControls'
import './ControlsPanel.css'

export default function ControlsPanel({
  mode,
  handleModeChange,
  prompt,
  setPrompt,
  targetId,
  setTargetId,
  autoTrackEnabled,
  setAutoTrackEnabled,
  handleAutoStart,
  handlePromptSubmit,
  handleClearPrompt,
  handleTargetIdSubmit
}) {
  return (
    <div className="controls-stack">
      <div className="controls-main">
        <PromptControls
            prompt={prompt}
            setPrompt={setPrompt}
            onSubmit={handlePromptSubmit}
            onClear={handleClearPrompt}
        />

        {mode === 'AUTO' && (
            <AutoControls
            prompt={prompt}
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
          />
        )}
      </div>

      <div className="controls-footer">
        <ModeSelector mode={mode} onChange={handleModeChange} />
      </div>
    </div>
  )
}