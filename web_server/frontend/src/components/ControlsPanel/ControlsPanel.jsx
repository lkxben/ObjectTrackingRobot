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
  targetId,
  annotated,
  setTargetId,
  autoTrackEnabled,
  setAutoTrackEnabled,
  handleAutoStart,
  handlePromptSubmit,
  handlePromptReset,
  handleTargetIdSubmit,
  handleTargetIdClear,
  toggleAnnotation
}) {
  return (
    <div className="controls-root">
      <div className="controls-side">
        <ModeSelector mode={mode} onChange={handleModeChange} />
      </div>

     <div className="controls-main">
        <div className="prompt-row">
          <div className="toggle-annotation-wrapper">
            <button
              className="toggle-annotation-btn"
              onClick={toggleAnnotation}
              title={annotated ? "Hide annotations" : "Show annotations"}
            >
              { annotated ? "Unannotate" : "Annotate" }
            </button>
          </div>

          <PromptControls
            prompt={prompt}
            onSubmit={handlePromptSubmit}
            onReset={handlePromptReset}
            annotated={annotated}
          />
        </div>

        {['AUTO_LOG', 'AUTO_TRACK', 'AUTO'].includes(mode) && (
          <AutoControls
            mode={mode}
            prompt={prompt}
            handlePromptReset={handlePromptReset}
            handleModeChange={handleModeChange}
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