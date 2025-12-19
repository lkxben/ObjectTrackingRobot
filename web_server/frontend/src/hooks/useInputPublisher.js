export default function useInputPublisher(inputTopicRef) {
  const sendInput = ({ mode, prompt, targetId, clearPrompt, clearTarget }) => {
    if (!inputTopicRef.current) return;

    const msg = {
      mode: mode ?? "",
      prompt: prompt ?? "",
      target_id: targetId !== undefined ? parseInt(targetId, 10) : -1,
      clear_prompt: !!clearPrompt,
      clear_target_id: !!clearTarget,
    };

    inputTopicRef.current.publish(msg);
  };

  return { sendInput };
}