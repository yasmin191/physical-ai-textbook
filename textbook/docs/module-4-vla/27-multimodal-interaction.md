---
sidebar_position: 7
title: "Chapter 27: Multi-Modal Interaction"
description: "Combine speech, gesture, and vision for natural human-robot interaction"
---

# Chapter 27: Multi-Modal Interaction

Humans communicate through multiple modalities simultaneously—speech, gesture, gaze, and facial expressions. This chapter covers integrating these modalities for natural, intuitive human-robot interaction.

## Learning Objectives

By the end of this chapter, you will be able to:

- Fuse information from multiple modalities
- Recognize and respond to human gestures
- Track human attention and gaze
- Combine speech with pointing gestures
- Design multi-modal interaction patterns

## 27.1 Multi-Modal Perception

### Modality Fusion Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                Multi-Modal Perception System                 │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐    │
│  │  Speech  │  │ Gesture  │  │   Gaze   │  │  Facial  │    │
│  │  (ASR)   │  │  (Pose)  │  │ (Track)  │  │  (Expr)  │    │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘    │
│       │             │             │             │           │
│       └─────────────┴──────┬──────┴─────────────┘           │
│                            │                                │
│                    ┌───────▼───────┐                        │
│                    │   Temporal    │                        │
│                    │   Alignment   │                        │
│                    └───────┬───────┘                        │
│                            │                                │
│                    ┌───────▼───────┐                        │
│                    │    Fusion     │                        │
│                    │    Module     │                        │
│                    └───────┬───────┘                        │
│                            │                                │
│                    ┌───────▼───────┐                        │
│                    │  Interaction  │                        │
│                    │    Intent     │                        │
│                    └───────────────┘                        │
└─────────────────────────────────────────────────────────────┘
```

### Temporal Alignment

```python
from collections import deque
import time

class TemporalAligner:
    """Align signals from different modalities in time."""
    
    def __init__(self, window_size=2.0):
        self.window_size = window_size  # seconds
        self.buffers = {
            'speech': deque(maxlen=100),
            'gesture': deque(maxlen=100),
            'gaze': deque(maxlen=100),
            'face': deque(maxlen=100)
        }
    
    def add_observation(self, modality: str, data: dict, timestamp: float = None):
        """Add observation from a modality."""
        if timestamp is None:
            timestamp = time.time()
        
        self.buffers[modality].append({
            'data': data,
            'timestamp': timestamp
        })
        
        # Clean old observations
        self._cleanup()
    
    def _cleanup(self):
        """Remove observations outside time window."""
        current_time = time.time()
        
        for modality in self.buffers:
            while (self.buffers[modality] and 
                   current_time - self.buffers[modality][0]['timestamp'] > self.window_size):
                self.buffers[modality].popleft()
    
    def get_aligned(self, reference_time: float, tolerance: float = 0.5):
        """Get observations aligned to a reference time."""
        aligned = {}
        
        for modality, buffer in self.buffers.items():
            # Find closest observation
            closest = None
            min_diff = float('inf')
            
            for obs in buffer:
                diff = abs(obs['timestamp'] - reference_time)
                if diff < min_diff and diff < tolerance:
                    min_diff = diff
                    closest = obs
            
            if closest:
                aligned[modality] = closest['data']
        
        return aligned
```

## 27.2 Gesture Recognition

### Body Pose Detection

```python
import mediapipe as mp
import numpy as np

class GestureRecognizer:
    """Recognize gestures from body pose."""
    
    def __init__(self):
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        # Gesture definitions
        self.gestures = {
            'pointing': self.detect_pointing,
            'waving': self.detect_waving,
            'beckoning': self.detect_beckoning,
            'stop': self.detect_stop,
            'thumbs_up': self.detect_thumbs_up
        }
    
    def process_frame(self, rgb_image):
        """Process image and detect gestures."""
        results = self.pose.process(rgb_image)
        
        if not results.pose_landmarks:
            return None
        
        landmarks = self.extract_landmarks(results.pose_landmarks)
        
        # Check each gesture
        detected_gestures = []
        for gesture_name, detector in self.gestures.items():
            confidence = detector(landmarks)
            if confidence > 0.7:
                detected_gestures.append({
                    'gesture': gesture_name,
                    'confidence': confidence,
                    'landmarks': landmarks
                })
        
        return detected_gestures
    
    def extract_landmarks(self, pose_landmarks):
        """Extract relevant landmarks."""
        landmarks = {}
        
        for idx, landmark in enumerate(pose_landmarks.landmark):
            name = self.mp_pose.PoseLandmark(idx).name
            landmarks[name] = np.array([landmark.x, landmark.y, landmark.z])
        
        return landmarks
    
    def detect_pointing(self, landmarks):
        """Detect pointing gesture."""
        # Get arm landmarks
        shoulder = landmarks.get('RIGHT_SHOULDER')
        elbow = landmarks.get('RIGHT_ELBOW')
        wrist = landmarks.get('RIGHT_WRIST')
        index_tip = landmarks.get('RIGHT_INDEX')
        
        if any(l is None for l in [shoulder, elbow, wrist, index_tip]):
            return 0.0
        
        # Check if arm is extended
        arm_length = np.linalg.norm(shoulder - elbow) + np.linalg.norm(elbow - wrist)
        direct_length = np.linalg.norm(shoulder - wrist)
        extension_ratio = direct_length / arm_length
        
        # Check if index is extended relative to wrist
        index_extension = np.linalg.norm(index_tip - wrist)
        
        if extension_ratio > 0.85 and index_extension > 0.05:
            return min(1.0, extension_ratio)
        
        return 0.0
    
    def detect_waving(self, landmarks):
        """Detect waving gesture."""
        wrist = landmarks.get('RIGHT_WRIST')
        shoulder = landmarks.get('RIGHT_SHOULDER')
        
        if wrist is None or shoulder is None:
            return 0.0
        
        # Waving: hand above shoulder, moving side to side
        if wrist[1] < shoulder[1]:  # y is inverted
            return 0.8
        
        return 0.0
    
    def detect_stop(self, landmarks):
        """Detect stop/halt gesture (palm forward)."""
        wrist = landmarks.get('RIGHT_WRIST')
        shoulder = landmarks.get('RIGHT_SHOULDER')
        elbow = landmarks.get('RIGHT_ELBOW')
        
        if any(l is None for l in [wrist, shoulder, elbow]):
            return 0.0
        
        # Arm raised with elbow bent
        if wrist[1] < shoulder[1] and elbow[1] < shoulder[1]:
            # Check arm is roughly perpendicular
            upper_arm = elbow - shoulder
            forearm = wrist - elbow
            
            angle = np.arccos(np.clip(
                np.dot(upper_arm, forearm) / 
                (np.linalg.norm(upper_arm) * np.linalg.norm(forearm)), -1, 1))
            
            if 1.2 < angle < 2.0:  # ~70-115 degrees
                return 0.85
        
        return 0.0
    
    def get_pointing_direction(self, landmarks):
        """Get 3D direction of pointing gesture."""
        shoulder = landmarks.get('RIGHT_SHOULDER')
        wrist = landmarks.get('RIGHT_WRIST')
        index_tip = landmarks.get('RIGHT_INDEX')
        
        if any(l is None for l in [shoulder, wrist, index_tip]):
            return None
        
        # Direction from wrist to fingertip
        direction = index_tip - wrist
        direction = direction / np.linalg.norm(direction)
        
        return direction
```

## 27.3 Gaze Tracking

### Attention Detection

```python
class GazeTracker:
    """Track human gaze direction and attention."""
    
    def __init__(self):
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.5
        )
        
        # Eye landmark indices
        self.LEFT_EYE = [362, 385, 387, 263, 373, 380]
        self.RIGHT_EYE = [33, 160, 158, 133, 153, 144]
        self.LEFT_IRIS = [474, 475, 476, 477]
        self.RIGHT_IRIS = [469, 470, 471, 472]
    
    def process_frame(self, rgb_image):
        """Process frame and estimate gaze."""
        results = self.face_mesh.process(rgb_image)
        
        if not results.multi_face_landmarks:
            return None
        
        face_landmarks = results.multi_face_landmarks[0]
        
        # Get gaze direction
        gaze = self.estimate_gaze(face_landmarks, rgb_image.shape)
        
        # Get head pose
        head_pose = self.estimate_head_pose(face_landmarks, rgb_image.shape)
        
        # Determine attention target
        attention = self.determine_attention(gaze, head_pose)
        
        return {
            'gaze_direction': gaze,
            'head_pose': head_pose,
            'attention': attention,
            'looking_at_robot': self.is_looking_at_robot(gaze, head_pose)
        }
    
    def estimate_gaze(self, landmarks, image_shape):
        """Estimate gaze direction from iris position."""
        h, w, _ = image_shape
        
        # Get iris centers
        left_iris = np.mean([[landmarks.landmark[i].x * w, 
                            landmarks.landmark[i].y * h] 
                           for i in self.LEFT_IRIS], axis=0)
        right_iris = np.mean([[landmarks.landmark[i].x * w,
                             landmarks.landmark[i].y * h]
                            for i in self.RIGHT_IRIS], axis=0)
        
        # Get eye centers
        left_eye_center = np.mean([[landmarks.landmark[i].x * w,
                                   landmarks.landmark[i].y * h]
                                  for i in self.LEFT_EYE], axis=0)
        right_eye_center = np.mean([[landmarks.landmark[i].x * w,
                                    landmarks.landmark[i].y * h]
                                   for i in self.RIGHT_EYE], axis=0)
        
        # Gaze offset from eye center
        left_offset = left_iris - left_eye_center
        right_offset = right_iris - right_eye_center
        
        # Average gaze direction
        gaze_offset = (left_offset + right_offset) / 2
        
        return gaze_offset
    
    def estimate_head_pose(self, landmarks, image_shape):
        """Estimate head pose (yaw, pitch, roll)."""
        # Use specific face landmarks for pose estimation
        h, w, _ = image_shape
        
        # Key points
        nose_tip = np.array([landmarks.landmark[4].x * w,
                           landmarks.landmark[4].y * h,
                           landmarks.landmark[4].z * w])
        chin = np.array([landmarks.landmark[152].x * w,
                        landmarks.landmark[152].y * h,
                        landmarks.landmark[152].z * w])
        left_eye = np.array([landmarks.landmark[33].x * w,
                            landmarks.landmark[33].y * h,
                            landmarks.landmark[33].z * w])
        right_eye = np.array([landmarks.landmark[263].x * w,
                             landmarks.landmark[263].y * h,
                             landmarks.landmark[263].z * w])
        
        # Calculate angles
        eye_line = right_eye - left_eye
        yaw = np.arctan2(eye_line[2], eye_line[0])
        
        face_vertical = chin - nose_tip
        pitch = np.arctan2(face_vertical[2], face_vertical[1])
        
        roll = np.arctan2(eye_line[1], eye_line[0])
        
        return {'yaw': yaw, 'pitch': pitch, 'roll': roll}
    
    def is_looking_at_robot(self, gaze, head_pose, threshold=0.3):
        """Check if human is looking at robot."""
        # Combine head pose and gaze direction
        combined_yaw = abs(head_pose['yaw'])
        combined_pitch = abs(head_pose['pitch'])
        
        # If head is facing forward and gaze is centered
        if combined_yaw < threshold and combined_pitch < threshold:
            if np.linalg.norm(gaze) < 20:  # pixels
                return True
        
        return False
    
    def determine_attention(self, gaze, head_pose):
        """Determine what the human is attending to."""
        if self.is_looking_at_robot(gaze, head_pose):
            return 'robot'
        elif head_pose['yaw'] > 0.5:
            return 'right'
        elif head_pose['yaw'] < -0.5:
            return 'left'
        elif head_pose['pitch'] > 0.3:
            return 'down'
        elif head_pose['pitch'] < -0.3:
            return 'up'
        
        return 'forward'
```

## 27.4 Multi-Modal Fusion

### Fusion Module

```python
class MultiModalFusion:
    """Fuse information from multiple modalities."""
    
    def __init__(self):
        self.temporal_aligner = TemporalAligner()
        self.gesture_recognizer = GestureRecognizer()
        self.gaze_tracker = GazeTracker()
    
    def process(self, speech_input=None, image_frame=None, timestamp=None):
        """Process all available modalities."""
        if timestamp is None:
            timestamp = time.time()
        
        results = {}
        
        # Process speech
        if speech_input:
            self.temporal_aligner.add_observation('speech', {
                'text': speech_input
            }, timestamp)
            results['speech'] = speech_input
        
        # Process vision
        if image_frame is not None:
            # Gesture recognition
            gestures = self.gesture_recognizer.process_frame(image_frame)
            if gestures:
                self.temporal_aligner.add_observation('gesture', gestures, timestamp)
                results['gestures'] = gestures
            
            # Gaze tracking
            gaze = self.gaze_tracker.process_frame(image_frame)
            if gaze:
                self.temporal_aligner.add_observation('gaze', gaze, timestamp)
                results['gaze'] = gaze
        
        # Get aligned observations
        aligned = self.temporal_aligner.get_aligned(timestamp)
        
        # Fuse modalities
        fused_intent = self.fuse_intent(aligned)
        
        return {
            'individual': results,
            'aligned': aligned,
            'fused_intent': fused_intent
        }
    
    def fuse_intent(self, aligned):
        """Fuse aligned observations into unified intent."""
        intent = {
            'action': None,
            'target': None,
            'confidence': 0.0
        }
        
        # Speech + Pointing fusion
        if 'speech' in aligned and 'gesture' in aligned:
            speech_text = aligned['speech'].get('text', '').lower()
            gestures = aligned['gesture']
            
            pointing = next((g for g in gestures if g['gesture'] == 'pointing'), None)
            
            if pointing and any(word in speech_text for word in ['that', 'this', 'there', 'it']):
                # Deictic reference with pointing
                direction = self.gesture_recognizer.get_pointing_direction(pointing['landmarks'])
                
                intent['action'] = 'refer'
                intent['target'] = {'type': 'pointed', 'direction': direction}
                intent['confidence'] = pointing['confidence']
        
        # Attention-based intent
        if 'gaze' in aligned:
            gaze = aligned['gaze']
            
            if gaze['looking_at_robot']:
                intent['attention'] = 'robot'
                intent['confidence'] = max(intent['confidence'], 0.8)
        
        # Gesture-only intents
        if 'gesture' in aligned and not intent['action']:
            gestures = aligned['gesture']
            
            if any(g['gesture'] == 'stop' for g in gestures):
                intent['action'] = 'stop'
                intent['confidence'] = 0.9
            elif any(g['gesture'] == 'waving' for g in gestures):
                intent['action'] = 'greet'
                intent['confidence'] = 0.8
            elif any(g['gesture'] == 'beckoning' for g in gestures):
                intent['action'] = 'come'
                intent['confidence'] = 0.85
        
        return intent
```

## 27.5 Speech + Gesture Integration

### Deictic Reference Resolution

```python
class DeicticResolver:
    """Resolve deictic references (this, that, there) using pointing."""
    
    def __init__(self, scene_understanding):
        self.scene = scene_understanding
    
    def resolve(self, speech_text: str, pointing_direction: np.ndarray, robot_pose):
        """Resolve deictic expression to scene object."""
        # Check for deictic words
        deictic_words = ['this', 'that', 'there', 'here', 'it']
        has_deictic = any(word in speech_text.lower() for word in deictic_words)
        
        if not has_deictic or pointing_direction is None:
            return None
        
        # Ray cast from human hand position along pointing direction
        # Transform to world frame
        pointing_world = self.transform_to_world(pointing_direction, robot_pose)
        
        # Find objects along pointing ray
        objects = self.scene.get_objects()
        candidates = []
        
        for obj in objects:
            # Check if object is along pointing direction
            obj_direction = obj['position'] - robot_pose['position']
            obj_direction = obj_direction / np.linalg.norm(obj_direction)
            
            alignment = np.dot(pointing_world, obj_direction)
            
            if alignment > 0.8:  # Within ~37 degrees
                distance = np.linalg.norm(obj['position'] - robot_pose['position'])
                candidates.append({
                    'object': obj,
                    'alignment': alignment,
                    'distance': distance
                })
        
        if not candidates:
            return None
        
        # Sort by alignment and proximity
        candidates.sort(key=lambda x: (-x['alignment'], x['distance']))
        
        return candidates[0]['object']
    
    def transform_to_world(self, direction, robot_pose):
        """Transform pointing direction to world frame."""
        # Rotate by robot orientation
        rotation = robot_pose.get('orientation_matrix', np.eye(3))
        return rotation @ direction
```

## 27.6 Interaction Patterns

### Multi-Modal Command Handler

```python
class MultiModalCommandHandler:
    """Handle commands from multiple modalities."""
    
    def __init__(self, fusion_module, conversation_agent, action_executor):
        self.fusion = fusion_module
        self.agent = conversation_agent
        self.executor = action_executor
        
        # Interaction state
        self.awaiting_confirmation = False
        self.pending_action = None
    
    def process_interaction(self, speech=None, image=None):
        """Process multi-modal interaction."""
        # Fuse modalities
        fused = self.fusion.process(speech, image)
        intent = fused['fused_intent']
        
        # Handle based on fused intent
        if intent['action'] == 'stop':
            return self.handle_stop()
        
        elif intent['action'] == 'refer':
            return self.handle_reference(speech, intent['target'])
        
        elif intent['action'] == 'greet':
            return self.handle_greeting()
        
        elif intent['action'] == 'come':
            return self.handle_summon()
        
        elif speech:
            # Fall back to speech-only processing
            return self.handle_speech_only(speech)
        
        elif fused['aligned'].get('gaze', {}).get('looking_at_robot'):
            # Human is looking at robot, be ready to engage
            return self.signal_ready()
        
        return None
    
    def handle_stop(self):
        """Handle stop gesture."""
        self.executor.emergency_stop()
        return "Stopping all actions."
    
    def handle_reference(self, speech, target):
        """Handle deictic reference."""
        # Resolve what "that" or "this" refers to
        pointed_object = target.get('resolved_object')
        
        if pointed_object:
            # Insert object name into speech understanding
            speech_with_object = speech.replace('that', pointed_object['name'])
            speech_with_object = speech_with_object.replace('this', pointed_object['name'])
            
            # Process as regular command
            response = self.agent.respond(speech_with_object)
            return response
        else:
            return "I'm not sure what you're pointing at. Could you be more specific?"
    
    def handle_greeting(self):
        """Handle wave greeting."""
        # Wave back
        self.executor.execute('GESTURE', ['wave'])
        return "Hello! How can I help you?"
    
    def handle_summon(self):
        """Handle beckoning gesture."""
        return "Coming to you."
    
    def handle_speech_only(self, speech):
        """Handle speech without gesture context."""
        return self.agent.respond(speech)
    
    def signal_ready(self):
        """Signal robot is ready to engage."""
        # Subtle acknowledgment - look at human, slight nod
        self.executor.execute('LOOK_AT', ['human'])
        return None  # Don't speak, just acknowledge visually
```

## 27.7 ROS 2 Multi-Modal Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

class MultiModalInteractionNode(Node):
    def __init__(self):
        super().__init__('multimodal_interaction')
        
        # Components
        self.fusion = MultiModalFusion()
        self.handler = MultiModalCommandHandler(
            self.fusion,
            ConversationAgent(),
            ActionExecutor()
        )
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw',
            self.image_callback, 10
        )
        self.speech_sub = self.create_subscription(
            String, '/voice/transcription',
            self.speech_callback, 10
        )
        
        # Publishers
        self.response_pub = self.create_publisher(String, '/response', 10)
        
        # State
        self.latest_image = None
        self.latest_speech = None
        self.speech_time = 0
        
        # Processing timer
        self.timer = self.create_timer(0.1, self.process_callback)
        
        self.get_logger().info('Multi-modal interaction node ready')
    
    def image_callback(self, msg):
        """Store latest image."""
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
    
    def speech_callback(self, msg):
        """Handle speech input."""
        self.latest_speech = msg.data
        self.speech_time = time.time()
    
    def process_callback(self):
        """Process multi-modal input."""
        if self.latest_image is None:
            return
        
        # Process with or without recent speech
        speech = None
        if self.latest_speech and time.time() - self.speech_time < 2.0:
            speech = self.latest_speech
            self.latest_speech = None
        
        # Process interaction
        response = self.handler.process_interaction(speech, self.latest_image)
        
        if response:
            msg = String()
            msg.data = response
            self.response_pub.publish(msg)
```

## 27.8 Summary

In this chapter, you learned:

- **Multi-modal perception**: Combining speech, gesture, gaze
- **Temporal alignment**: Synchronizing modality streams
- **Gesture recognition**: Detecting pointing, waving, stop
- **Gaze tracking**: Attention and intention from eyes
- **Fusion**: Combining modalities for unified intent
- **Deictic resolution**: Resolving "this" and "that" with pointing

Multi-modal interaction makes humanoids more natural to work with. In the final chapter, we'll bring everything together for a capstone project.

## Review Questions

1. Why is temporal alignment important for multi-modal fusion?
2. How do you detect a pointing gesture?
3. What is deictic reference resolution?
4. How does gaze inform interaction intent?
5. When should speech and gesture be combined?

## Hands-On Exercise

1. Implement gesture recognition with MediaPipe
2. Add gaze tracking for attention detection
3. Create a temporal alignment buffer
4. Implement speech + pointing fusion
5. Build a multi-modal ROS node
6. Test with natural multi-modal commands
