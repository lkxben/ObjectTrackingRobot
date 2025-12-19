import React from 'react';
import './PromptControls.css'

function PromptControls({ prompt, setPrompt, onSubmit, onClear }) {
  return (
    <div className="prompt-inputs">
      <input
        type="text"
        placeholder="Enter prompts, comma-separated"
        value={prompt}
        onChange={e => setPrompt(e.target.value)}
      />
      <button onClick={onSubmit}>Prompt</button>
      <button onClick={onClear}>Clear</button>
    </div>
  );
}

export default PromptControls;