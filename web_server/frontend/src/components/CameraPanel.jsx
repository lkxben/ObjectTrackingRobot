import React from 'react';
import ModeSelector from './ModeSelector';

function CameraPanel({
  imgRef,
  streamActive,
  fps,
  mode,
  onModeChange,
  children
}) {
  return (
    <section className="camera-card">
      <div className="camera-image-wrapper">
        {streamActive ? (
          <img ref={imgRef} src="" alt="Camera stream" />
        ) : (
          <div className="placeholder-text">Waiting for stream...</div>
        )}
      </div>

      <div className="camera-status">
        <span>FPS: {fps}</span>
      </div>

      <ModeSelector
        mode={mode}
        onChange={onModeChange}
      />

      {children}
    </section>
  );
}

export default CameraPanel;