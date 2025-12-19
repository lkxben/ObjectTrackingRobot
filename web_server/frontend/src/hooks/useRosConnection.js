import { useRef, useEffect } from 'react';
import ROSLIB from 'roslib';

export default function useRosConnection(url = 'wss://objecttrackingrobot.onrender.com/rosbridge?client=frontend') {
  const rosRef = useRef(null);
  const cameraTopicRef = useRef(null);
  const inputTopicRef = useRef(null);
  const logTopicRef = useRef(null);
  const manualTopicRef = useRef(null);

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url });
    rosRef.current = ros;

    inputTopicRef.current = new ROSLIB.Topic({
      ros,
      name: '/input',
      messageType: 'robot_msgs/Input',
    });

    logTopicRef.current = new ROSLIB.Topic({
      ros,
      name: '/turret/log',
      messageType: 'robot_msgs/TurretLog',
    });

    manualTopicRef.current = new ROSLIB.Topic({
      ros,
      name: '/motor/manual',
      messageType: 'std_msgs/Float32',
    });

    cameraTopicRef.current = new ROSLIB.Topic({
      ros,
      name: '/camera/annotated/compressed',
      messageType: 'sensor_msgs/CompressedImage',
    });

    const heartbeatTopic = new ROSLIB.Topic({
      ros,
      name: '/viewer/heartbeat',
      messageType: 'std_msgs/Empty',
    });

    const interval = setInterval(() => heartbeatTopic.publish(new ROSLIB.Message({})), 30000);

    return () => clearInterval(interval);
  }, [url]);

  return {
    rosRef,
    cameraTopicRef,
    inputTopicRef,
    logTopicRef,
    manualTopicRef,
  };
}