---
sidebar_position: 6
title: "Chapter 26: Conversational Robotics"
description: "Build dialogue systems for natural human-robot interaction"
---

# Chapter 26: Conversational Robotics

Humanoid robots should engage in natural conversations with humans. This chapter covers building dialogue systems that maintain context, handle multi-turn conversations, and integrate with robot actions.

## Learning Objectives

By the end of this chapter, you will be able to:

- Design dialogue systems for robots
- Manage conversation context and state
- Handle multi-turn interactions
- Integrate dialogue with robot actions
- Implement proactive communication

## 26.1 Dialogue System Architecture

### Components

```
┌─────────────────────────────────────────────────────────────┐
│                  Conversational Robot System                 │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────┐     ┌─────────────┐     ┌─────────────┐   │
│  │    ASR      │────►│   Dialogue  │────►│    TTS      │   │
│  │  (Whisper)  │     │   Manager   │     │  (Speech)   │   │
│  └─────────────┘     └──────┬──────┘     └─────────────┘   │
│                             │                               │
│         ┌───────────────────┼───────────────────┐          │
│         ▼                   ▼                   ▼          │
│  ┌─────────────┐     ┌─────────────┐     ┌─────────────┐   │
│  │   Context   │     │    LLM      │     │   Action    │   │
│  │   Manager   │     │   (GPT-4)   │     │  Executor   │   │
│  └─────────────┘     └─────────────┘     └─────────────┘   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Dialogue Manager

```python
from dataclasses import dataclass, field
from typing import List, Dict, Optional
from enum import Enum

class DialogueState(Enum):
    IDLE = "idle"
    LISTENING = "listening"
    PROCESSING = "processing"
    RESPONDING = "responding"
    EXECUTING = "executing"
    WAITING_CONFIRMATION = "waiting_confirmation"

@dataclass
class ConversationTurn:
    speaker: str  # "user" or "robot"
    text: str
    timestamp: float
    intent: Optional[str] = None
    entities: Optional[Dict] = None
    action_taken: Optional[str] = None

@dataclass
class ConversationContext:
    turns: List[ConversationTurn] = field(default_factory=list)
    current_task: Optional[str] = None
    pending_actions: List[Dict] = field(default_factory=list)
    user_preferences: Dict = field(default_factory=dict)
    last_topic: Optional[str] = None
    
    def add_turn(self, speaker: str, text: str, **kwargs):
        turn = ConversationTurn(
            speaker=speaker,
            text=text,
            timestamp=time.time(),
            **kwargs
        )
        self.turns.append(turn)
        
        # Keep only recent turns
        if len(self.turns) > 20:
            self.turns = self.turns[-20:]
    
    def get_history_text(self, num_turns=10):
        """Get conversation history as text."""
        recent = self.turns[-num_turns:]
        lines = []
        for turn in recent:
            prefix = "User" if turn.speaker == "user" else "Robot"
            lines.append(f"{prefix}: {turn.text}")
        return "\n".join(lines)
```

## 26.2 LLM-Based Dialogue

### Conversation Agent

```python
from openai import OpenAI

class ConversationAgent:
    def __init__(self):
        self.client = OpenAI()
        self.context = ConversationContext()
        
        self.system_prompt = """You are a helpful humanoid robot assistant named Atlas.

Personality:
- Friendly and helpful
- Concise but informative
- Safety-conscious
- Admits limitations honestly

Capabilities you can perform:
- Navigate to rooms (kitchen, living room, bedroom, bathroom)
- Pick up and move objects
- Open and close doors/drawers
- Answer questions
- Set reminders

When the user asks you to do something physical:
1. Confirm understanding
2. Ask for clarification if needed
3. Respond with [ACTION: action_name(parameters)] when ready to act

Example:
User: Can you bring me my glasses?
Robot: I'd be happy to help! Do you know where your glasses are?
User: I think they're on the kitchen counter.
Robot: Got it. I'll go to the kitchen and look for your glasses on the counter.
[ACTION: navigate(kitchen)]
[ACTION: find_object(glasses)]
[ACTION: pick_up(glasses)]
[ACTION: navigate(living_room)]
[ACTION: give_to_user(glasses)]
"""
    
    def respond(self, user_input: str, robot_state: dict = None):
        """Generate response to user input."""
        # Add user turn
        self.context.add_turn("user", user_input)
        
        # Build messages
        messages = [{"role": "system", "content": self.system_prompt}]
        
        # Add robot state if available
        if robot_state:
            state_msg = self._format_robot_state(robot_state)
            messages.append({"role": "system", "content": f"Current state: {state_msg}"})
        
        # Add conversation history
        for turn in self.context.turns[-10:]:
            role = "user" if turn.speaker == "user" else "assistant"
            messages.append({"role": role, "content": turn.text})
        
        # Generate response
        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=messages,
            temperature=0.7,
            max_tokens=300
        )
        
        response_text = response.choices[0].message.content
        
        # Parse actions from response
        actions = self._extract_actions(response_text)
        clean_response = self._clean_response(response_text)
        
        # Add robot turn
        self.context.add_turn("robot", clean_response, action_taken=str(actions) if actions else None)
        
        return {
            'text': clean_response,
            'actions': actions
        }
    
    def _extract_actions(self, text):
        """Extract action commands from response."""
        import re
        actions = []
        pattern = r'\[ACTION:\s*(\w+)\((.*?)\)\]'
        
        matches = re.findall(pattern, text)
        for action, params in matches:
            param_list = [p.strip() for p in params.split(',') if p.strip()]
            actions.append({
                'action': action,
                'parameters': param_list
            })
        
        return actions
    
    def _clean_response(self, text):
        """Remove action tags from response."""
        import re
        return re.sub(r'\[ACTION:.*?\]', '', text).strip()
    
    def _format_robot_state(self, state):
        """Format robot state for context."""
        parts = []
        parts.append(f"Location: {state.get('location', 'unknown')}")
        parts.append(f"Holding: {state.get('holding', 'nothing')}")
        parts.append(f"Battery: {state.get('battery', '?')}%")
        return ", ".join(parts)
```

## 26.3 Multi-Turn Dialogue Management

### Slot Filling

```python
class SlotFillingDialogue:
    """Manage slot-filling dialogues for tasks requiring multiple pieces of info."""
    
    def __init__(self):
        self.task_templates = {
            'fetch_object': {
                'slots': {
                    'object': {'required': True, 'prompt': "What would you like me to fetch?"},
                    'location': {'required': False, 'prompt': "Do you know where it is?"},
                    'destination': {'required': False, 'prompt': "Where should I bring it?"}
                }
            },
            'set_reminder': {
                'slots': {
                    'message': {'required': True, 'prompt': "What should I remind you about?"},
                    'time': {'required': True, 'prompt': "When should I remind you?"}
                }
            },
            'navigate': {
                'slots': {
                    'destination': {'required': True, 'prompt': "Where would you like me to go?"}
                }
            }
        }
        
        self.current_task = None
        self.filled_slots = {}
    
    def start_task(self, task_type: str, initial_slots: dict = None):
        """Start a new slot-filling task."""
        if task_type not in self.task_templates:
            return None
        
        self.current_task = task_type
        self.filled_slots = initial_slots or {}
        
        return self.get_next_prompt()
    
    def fill_slot(self, slot_name: str, value: str):
        """Fill a slot with a value."""
        self.filled_slots[slot_name] = value
    
    def get_next_prompt(self):
        """Get prompt for next unfilled required slot."""
        if not self.current_task:
            return None
        
        template = self.task_templates[self.current_task]
        
        for slot_name, slot_info in template['slots'].items():
            if slot_info['required'] and slot_name not in self.filled_slots:
                return slot_info['prompt']
        
        return None  # All required slots filled
    
    def is_complete(self):
        """Check if all required slots are filled."""
        if not self.current_task:
            return False
        
        template = self.task_templates[self.current_task]
        
        for slot_name, slot_info in template['slots'].items():
            if slot_info['required'] and slot_name not in self.filled_slots:
                return False
        
        return True
    
    def get_task_parameters(self):
        """Get filled slots as task parameters."""
        return self.filled_slots.copy()
```

### Clarification Handling

```python
class ClarificationHandler:
    """Handle ambiguous user inputs."""
    
    def __init__(self, conversation_agent):
        self.agent = conversation_agent
    
    def check_ambiguity(self, user_input: str, parsed_intent: dict):
        """Check if clarification is needed."""
        ambiguities = []
        
        # Check for ambiguous references
        if self._has_ambiguous_reference(user_input):
            ambiguities.append({
                'type': 'reference',
                'question': "Which one do you mean?"
            })
        
        # Check for missing required info
        if parsed_intent.get('intent') == 'fetch_object':
            if not parsed_intent.get('object'):
                ambiguities.append({
                    'type': 'missing_object',
                    'question': "What would you like me to get?"
                })
        
        # Check for location ambiguity
        if 'there' in user_input.lower() or 'here' in user_input.lower():
            ambiguities.append({
                'type': 'location',
                'question': "Could you be more specific about the location?"
            })
        
        return ambiguities
    
    def _has_ambiguous_reference(self, text):
        """Check for ambiguous pronouns/references."""
        ambiguous_words = ['it', 'that', 'this', 'those', 'them', 'one']
        text_lower = text.lower()
        
        for word in ambiguous_words:
            if f" {word} " in f" {text_lower} ":
                # Check if context resolves the reference
                if not self._can_resolve_reference(word):
                    return True
        
        return False
    
    def _can_resolve_reference(self, reference):
        """Check if reference can be resolved from context."""
        # Look at recent context for antecedent
        recent_turns = self.agent.context.turns[-5:]
        
        for turn in recent_turns:
            if turn.entities and 'object' in turn.entities:
                return True
        
        return False
    
    def generate_clarification(self, ambiguity):
        """Generate clarification question."""
        templates = {
            'reference': "I want to make sure I understand correctly. {}",
            'missing_object': "{}",
            'location': "{}"
        }
        
        template = templates.get(ambiguity['type'], "{}")
        return template.format(ambiguity['question'])
```

## 26.4 Context-Aware Responses

### Entity Tracking

```python
class EntityTracker:
    """Track entities mentioned in conversation."""
    
    def __init__(self):
        self.entities = {}  # name -> info
        self.recent_mentions = []  # ordered by recency
    
    def update(self, turn_text: str, parsed_entities: dict):
        """Update entity tracking with new turn."""
        for entity_type, entity_value in parsed_entities.items():
            self.entities[entity_value] = {
                'type': entity_type,
                'last_mentioned': time.time()
            }
            
            # Update recency
            if entity_value in self.recent_mentions:
                self.recent_mentions.remove(entity_value)
            self.recent_mentions.insert(0, entity_value)
    
    def resolve_reference(self, reference: str):
        """Resolve a reference like 'it' or 'that'."""
        reference_lower = reference.lower()
        
        if reference_lower in ['it', 'that', 'this']:
            # Return most recent entity
            if self.recent_mentions:
                return self.recent_mentions[0]
        
        elif reference_lower in ['them', 'those']:
            # Return recent plural entities
            return [e for e in self.recent_mentions[:3] 
                    if self.entities[e].get('plural', False)]
        
        return None
    
    def get_entity_of_type(self, entity_type: str):
        """Get most recent entity of a specific type."""
        for entity_name in self.recent_mentions:
            if self.entities[entity_name]['type'] == entity_type:
                return entity_name
        return None
```

### Conversation Memory

```python
class ConversationMemory:
    """Long-term memory for conversation context."""
    
    def __init__(self, db_path="conversation_memory.db"):
        import sqlite3
        self.conn = sqlite3.connect(db_path)
        self._init_db()
    
    def _init_db(self):
        """Initialize database tables."""
        self.conn.execute("""
            CREATE TABLE IF NOT EXISTS user_preferences (
                user_id TEXT,
                preference_key TEXT,
                preference_value TEXT,
                updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                PRIMARY KEY (user_id, preference_key)
            )
        """)
        
        self.conn.execute("""
            CREATE TABLE IF NOT EXISTS conversation_facts (
                id INTEGER PRIMARY KEY,
                user_id TEXT,
                fact_type TEXT,
                fact_content TEXT,
                confidence REAL,
                learned_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        """)
        
        self.conn.commit()
    
    def remember_preference(self, user_id: str, key: str, value: str):
        """Store a user preference."""
        self.conn.execute("""
            INSERT OR REPLACE INTO user_preferences (user_id, preference_key, preference_value)
            VALUES (?, ?, ?)
        """, (user_id, key, value))
        self.conn.commit()
    
    def get_preference(self, user_id: str, key: str):
        """Retrieve a user preference."""
        cursor = self.conn.execute("""
            SELECT preference_value FROM user_preferences
            WHERE user_id = ? AND preference_key = ?
        """, (user_id, key))
        
        row = cursor.fetchone()
        return row[0] if row else None
    
    def learn_fact(self, user_id: str, fact_type: str, content: str, confidence: float = 1.0):
        """Learn a new fact from conversation."""
        self.conn.execute("""
            INSERT INTO conversation_facts (user_id, fact_type, fact_content, confidence)
            VALUES (?, ?, ?, ?)
        """, (user_id, fact_type, content, confidence))
        self.conn.commit()
    
    def get_relevant_facts(self, user_id: str, context: str, limit: int = 5):
        """Get facts relevant to current context."""
        # Simple keyword matching; could use embeddings
        cursor = self.conn.execute("""
            SELECT fact_type, fact_content, confidence
            FROM conversation_facts
            WHERE user_id = ?
            ORDER BY learned_at DESC
            LIMIT ?
        """, (user_id, limit * 2))
        
        facts = cursor.fetchall()
        
        # Filter by relevance
        relevant = []
        context_lower = context.lower()
        for fact_type, content, confidence in facts:
            if any(word in context_lower for word in content.lower().split()):
                relevant.append({
                    'type': fact_type,
                    'content': content,
                    'confidence': confidence
                })
        
        return relevant[:limit]
```

## 26.5 Proactive Communication

### Proactive Agent

```python
class ProactiveAgent:
    """Agent that initiates conversation when appropriate."""
    
    def __init__(self, robot_interface, conversation_agent):
        self.robot = robot_interface
        self.agent = conversation_agent
        
        self.triggers = []
        self._setup_triggers()
    
    def _setup_triggers(self):
        """Set up proactive communication triggers."""
        self.triggers = [
            {
                'condition': self.check_human_entered,
                'response': self.greet_human
            },
            {
                'condition': self.check_task_complete,
                'response': self.announce_completion
            },
            {
                'condition': self.check_needs_help,
                'response': self.offer_help
            },
            {
                'condition': self.check_reminder_due,
                'response': self.deliver_reminder
            },
            {
                'condition': self.check_anomaly,
                'response': self.report_anomaly
            }
        ]
    
    def check_triggers(self):
        """Check all triggers and respond if any fire."""
        for trigger in self.triggers:
            if trigger['condition']():
                return trigger['response']()
        return None
    
    def check_human_entered(self):
        """Check if a human just entered the room."""
        return self.robot.perception.human_just_entered()
    
    def greet_human(self):
        """Greet human who entered."""
        person = self.robot.perception.get_detected_person()
        
        if person and person.get('known'):
            name = person['name']
            return f"Hello {name}! How can I help you today?"
        else:
            return "Hello! I'm Atlas. Let me know if you need any help."
    
    def check_task_complete(self):
        """Check if a task just completed."""
        return self.robot.task_manager.task_just_completed()
    
    def announce_completion(self):
        """Announce task completion."""
        task = self.robot.task_manager.get_last_completed_task()
        return f"I've finished {task['description']}."
    
    def check_needs_help(self):
        """Check if human appears to need help."""
        # Detect struggling behavior
        return self.robot.perception.detect_struggling()
    
    def offer_help(self):
        """Offer assistance."""
        return "You look like you might need some help. Is there something I can do for you?"
    
    def check_reminder_due(self):
        """Check if a reminder is due."""
        return self.robot.reminders.has_due_reminder()
    
    def deliver_reminder(self):
        """Deliver a due reminder."""
        reminder = self.robot.reminders.get_due_reminder()
        return f"Reminder: {reminder['message']}"
    
    def check_anomaly(self):
        """Check for anomalies to report."""
        return self.robot.perception.detected_anomaly()
    
    def report_anomaly(self):
        """Report detected anomaly."""
        anomaly = self.robot.perception.get_anomaly()
        return f"I noticed something unusual: {anomaly['description']}. Should I take a closer look?"
```

## 26.6 ROS 2 Dialogue Node

### Complete Dialogue Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from humanoid_msgs.msg import DialogueTurn
from humanoid_msgs.srv import Chat

class DialogueNode(Node):
    def __init__(self):
        super().__init__('dialogue_node')
        
        # Components
        self.agent = ConversationAgent()
        self.clarification = ClarificationHandler(self.agent)
        self.entity_tracker = EntityTracker()
        self.memory = ConversationMemory()
        self.proactive = ProactiveAgent(self.get_robot_interface(), self.agent)
        
        # Subscribers
        self.speech_sub = self.create_subscription(
            String, '/voice/transcription',
            self.speech_callback, 10
        )
        
        # Publishers
        self.response_pub = self.create_publisher(String, '/dialogue/response', 10)
        self.action_pub = self.create_publisher(String, '/dialogue/action', 10)
        
        # Service
        self.chat_service = self.create_service(Chat, '/chat', self.chat_callback)
        
        # Proactive timer
        self.proactive_timer = self.create_timer(5.0, self.proactive_check)
        
        self.get_logger().info('Dialogue node ready')
    
    def speech_callback(self, msg):
        """Handle incoming speech."""
        user_input = msg.data
        self.process_input(user_input)
    
    def chat_callback(self, request, response):
        """Handle chat service requests."""
        result = self.process_input(request.message)
        response.response = result['text']
        response.actions = str(result.get('actions', []))
        return response
    
    def process_input(self, user_input: str):
        """Process user input and generate response."""
        self.get_logger().info(f"User: {user_input}")
        
        # Get robot state
        robot_state = self.get_robot_state()
        
        # Check for clarification needs
        # (would normally parse intent first)
        
        # Generate response
        result = self.agent.respond(user_input, robot_state)
        
        # Publish response
        response_msg = String()
        response_msg.data = result['text']
        self.response_pub.publish(response_msg)
        
        # Publish actions if any
        if result['actions']:
            for action in result['actions']:
                action_msg = String()
                action_msg.data = f"{action['action']}({','.join(action['parameters'])})"
                self.action_pub.publish(action_msg)
        
        self.get_logger().info(f"Robot: {result['text']}")
        
        return result
    
    def proactive_check(self):
        """Check for proactive communication opportunities."""
        message = self.proactive.check_triggers()
        
        if message:
            self.get_logger().info(f"Proactive: {message}")
            
            response_msg = String()
            response_msg.data = message
            self.response_pub.publish(response_msg)
            
            # Add to context
            self.agent.context.add_turn("robot", message)
    
    def get_robot_state(self):
        """Get current robot state."""
        # Would query actual robot state
        return {
            'location': 'living_room',
            'holding': None,
            'battery': 85
        }
    
    def get_robot_interface(self):
        """Get robot interface for proactive agent."""
        # Return actual robot interface
        return None

def main():
    rclpy.init()
    node = DialogueNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## 26.7 Summary

In this chapter, you learned:

- **Dialogue architecture**: Components and data flow
- **LLM dialogue**: Using GPT-4 for conversation
- **Multi-turn management**: Slot filling and clarification
- **Context tracking**: Entities and memory
- **Proactive communication**: Robot-initiated dialogue
- **ROS integration**: Complete dialogue node

Conversational capabilities make humanoids more natural to interact with. In the next chapter, we'll combine all modalities for multi-modal interaction.

## Review Questions

1. What components make up a dialogue system?
2. How do you handle multi-turn conversations?
3. What is slot filling and when is it used?
4. How does entity tracking improve dialogue?
5. When should a robot initiate conversation?

## Hands-On Exercise

1. Implement a conversation agent with context
2. Add slot-filling for a fetch task
3. Create an entity tracker
4. Implement proactive greetings
5. Build a complete dialogue ROS node
6. Test multi-turn conversations
