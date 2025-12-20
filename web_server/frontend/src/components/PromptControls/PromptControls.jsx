import React, { useEffect, useState } from 'react'
import './PromptControls.css'

function PromptControls({ prompt, setPrompt, onSubmit, onReset }) {
  const [inputValue, setInputValue] = useState('')

  // Keep input in sync with external prompt reset
  useEffect(() => {
    if (prompt === '') setInputValue('')
  }, [prompt])

  const applied = prompt !== ''

  const handleButton = () => {
    const trimmed = inputValue.trim()
    if (!applied && trimmed !== '') {
      setPrompt(trimmed)
      onSubmit()
    } else {
      onReset()
      setPrompt('')
      setInputValue('')
    }
  }

  const handleKeyDown = (e) => {
    if (e.key === 'Enter') handleButton()
  }

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
  )
}

export default PromptControls