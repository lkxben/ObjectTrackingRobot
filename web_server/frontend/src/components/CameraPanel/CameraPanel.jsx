import React from 'react';
import './CameraPanel.css';

function CameraPanel({ imgRef, streamActive, fps }) {
  return (
    <div className="camera-card">
      <div className="camera-image-wrapper">
        {streamActive ? (
          <img ref={imgRef} src="" alt="Camera stream" />
        ) : (
          <div className="placeholder-text">Waiting for stream...</div>
        )}
        {streamActive && <div className="camera-fps">FPS: {fps}</div>}
      </div>
    </div>
  );
}

export default CameraPanel;