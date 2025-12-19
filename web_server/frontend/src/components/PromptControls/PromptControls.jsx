import React, { useRef } from 'react';
import './PromptControls.css'

function PromptControls({ prompt, setPrompt, onSubmit, onReset }) {
  const applied = prompt !== "";
  const inputRef = useRef(null);

  const handleButton = () => {
    if (!applied && inputRef.current.value.trim() !== '') {
      const value = inputRef.current.value;
      setPrompt(value);
      onSubmit();
    } else {
      onReset();
      setPrompt('');
      inputRef.current.value = '';
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
          ref={inputRef}
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