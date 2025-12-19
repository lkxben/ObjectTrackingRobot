import { useEffect, useState } from 'react'
import './TopBar'

export default function TopBar() {
  const [time, setTime] = useState(() => new Date())

  useEffect(() => {
    const id = setInterval(() => {
      setTime(new Date())
    }, 1000)

    return () => clearInterval(id)
  }, [])

  return (
    <header className="top-bar">
      <div className="top-bar-left">
        PiSentinel Dashboard
      </div>

      <div className="top-bar-right">
        {time.toLocaleTimeString()}
      </div>
    </header>
  )
}