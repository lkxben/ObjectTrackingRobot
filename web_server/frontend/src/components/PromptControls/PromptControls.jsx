import React, { useEffect, useState } from 'react';
import './PromptControls.css';

function PromptControls({ prompt, onSubmit, onReset }) {
  const [inputValue, setInputValue] = useState('');
  const [applied, setApplied] = useState(false);

  // Keep input in sync if prompt is externally reset
  useEffect(() => {
    if (prompt === '') {
      setInputValue('');
      setApplied(false);
    }
  }, [prompt]);

  const handleButton = () => {
    const trimmed = inputValue.trim();

    if (!applied && trimmed !== '') {
      onSubmit(trimmed);
      setApplied(true);
    } else if (applied) {
      onReset();
      setInputValue('');
      setApplied(false);
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter') handleButton();
  };

  return (
    <div className="prompt-inputs">
      <div className="input-wrapper">
        <input
          type="text"
          placeholder="Filter annotations with prompts"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyDown={handleKeyDown}
        />
        <button className="prompt-btn" onClick={handleButton}>
          {applied ? 'Reset' : 'Prompt'}
        </button>
      </div>
    </div>
  );
}

export default PromptControls;