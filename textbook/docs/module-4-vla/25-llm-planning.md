---
sidebar_position: 5
title: "Chapter 25: Cognitive Planning with LLMs"
description: "Use large language models for task planning and reasoning in humanoid robots"
---

# Chapter 25: Cognitive Planning with LLMs

Large Language Models (LLMs) can serve as cognitive engines for humanoid robots, enabling high-level task planning, common-sense reasoning, and natural language understanding. This chapter explores integrating LLMs like GPT-4 for robot planning and decision-making.

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand how LLMs can augment robot intelligence
- Design prompts for robot task planning
- Implement LLM-based action planning
- Ground language in robot capabilities
- Handle planning failures and replanning

## 25.1 LLMs as Robot Brains

### The Planning Stack

```
┌─────────────────────────────────────────────────────────────┐
│                    Cognitive Architecture                    │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │                    LLM (GPT-4)                       │   │
│  │  • Task understanding                               │   │
│  │  • Plan generation                                  │   │
│  │  • Common-sense reasoning                           │   │
│  └──────────────────────┬──────────────────────────────┘   │
│                         │                                   │
│                         ▼                                   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │              Action Grounding Layer                  │   │
│  │  • Map language to robot skills                     │   │
│  │  • Verify feasibility                               │   │
│  │  • Handle constraints                               │   │
│  └──────────────────────┬──────────────────────────────┘   │
│                         │                                   │
│                         ▼                                   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │              Robot Skill Library                     │   │
│  │  • Navigation • Manipulation • Locomotion           │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Why LLMs for Robotics?

| Capability | Traditional Planning | LLM Planning |
|------------|---------------------|--------------|
| Task understanding | Hand-coded | Natural language |
| Common sense | Limited ontologies | Emergent |
| Generalization | Domain-specific | Broad |
| Error recovery | Predefined | Adaptive |
| Novel tasks | Requires engineering | Zero-shot |

## 25.2 Setting Up LLM Integration

### OpenAI API Integration

```python
import openai
from openai import OpenAI
import os

class LLMPlanner:
    def __init__(self, model="gpt-4"):
        self.client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))
        self.model = model
        
        # System prompt defining robot capabilities
        self.system_prompt = """You are an AI planner for a humanoid robot. 
        
The robot has the following capabilities:
- NAVIGATE(location): Move to a named location
- PICK_UP(object): Pick up an object
- PLACE(object, location): Place an object at a location
- OPEN(object): Open a door, drawer, or container
- CLOSE(object): Close a door, drawer, or container
- SAY(message): Speak a message
- WAVE(): Wave hand in greeting
- LOOK_AT(target): Turn to look at something
- WAIT(seconds): Wait for specified time

When given a task, output a step-by-step plan using ONLY these actions.
Format each step as: STEP N: ACTION(parameters)

Consider:
- Physical constraints (can't pick up something already holding)
- Logical order (need to go to object before picking it up)
- Safety (announce intentions to humans nearby)
"""
    
    def generate_plan(self, task_description, context=None):
        """Generate a plan for the given task."""
        messages = [
            {"role": "system", "content": self.system_prompt}
        ]
        
        if context:
            messages.append({
                "role": "user",
                "content": f"Current context:\n{context}"
            })
        
        messages.append({
            "role": "user",
            "content": f"Task: {task_description}\n\nGenerate a plan:"
        })
        
        response = self.client.chat.completions.create(
            model=self.model,
            messages=messages,
            temperature=0.2,  # Low temperature for consistent planning
            max_tokens=500
        )
        
        plan_text = response.choices[0].message.content
        return self.parse_plan(plan_text)
    
    def parse_plan(self, plan_text):
        """Parse LLM output into structured plan."""
        import re
        
        steps = []
        pattern = r'STEP\s*(\d+):\s*(\w+)\((.*?)\)'
        
        matches = re.findall(pattern, plan_text, re.IGNORECASE)
        
        for step_num, action, params in matches:
            # Parse parameters
            param_list = [p.strip().strip('"\'') for p in params.split(',') if p.strip()]
            
            steps.append({
                'step': int(step_num),
                'action': action.upper(),
                'parameters': param_list
            })
        
        return steps
```

### ROS 2 LLM Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from humanoid_msgs.msg import Plan, PlanStep
from humanoid_msgs.srv import GeneratePlan

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner')
        
        self.planner = LLMPlanner()
        
        # Service for plan generation
        self.plan_service = self.create_service(
            GeneratePlan,
            '/generate_plan',
            self.generate_plan_callback
        )
        
        # Publisher for plans
        self.plan_pub = self.create_publisher(Plan, '/planned_actions', 10)
        
        # Subscriber for task requests
        self.task_sub = self.create_subscription(
            String,
            '/task_request',
            self.task_callback,
            10
        )
        
        self.get_logger().info('LLM Planner node ready')
    
    def generate_plan_callback(self, request, response):
        """Service callback for plan generation."""
        task = request.task_description
        context = request.context if request.context else None
        
        try:
            steps = self.planner.generate_plan(task, context)
            
            response.success = True
            response.plan = self.steps_to_plan_msg(steps)
            
        except Exception as e:
            self.get_logger().error(f'Planning failed: {e}')
            response.success = False
            response.error_message = str(e)
        
        return response
    
    def task_callback(self, msg):
        """Handle incoming task requests."""
        task = msg.data
        self.get_logger().info(f'Received task: {task}')
        
        # Get current context
        context = self.get_robot_context()
        
        # Generate plan
        steps = self.planner.generate_plan(task, context)
        
        # Publish plan
        plan_msg = self.steps_to_plan_msg(steps)
        self.plan_pub.publish(plan_msg)
    
    def get_robot_context(self):
        """Get current robot state as context."""
        # This would query actual robot state
        return """
        Robot location: living room
        Holding: nothing
        Nearby objects: cup, book, remote
        Nearby locations: kitchen, bedroom, front door
        People present: 1 person on couch
        """
    
    def steps_to_plan_msg(self, steps):
        """Convert parsed steps to ROS message."""
        plan = Plan()
        plan.header.stamp = self.get_clock().now().to_msg()
        
        for step in steps:
            step_msg = PlanStep()
            step_msg.step_number = step['step']
            step_msg.action = step['action']
            step_msg.parameters = step['parameters']
            plan.steps.append(step_msg)
        
        return plan
```

## 25.3 Prompt Engineering for Robots

### Capability Description

```python
ROBOT_CAPABILITIES = """
## Robot Capabilities

### Navigation
- NAVIGATE(location): Move to a location. Known locations: kitchen, living_room, bedroom, bathroom, front_door
- APPROACH(object): Move close to an object to interact with it

### Manipulation  
- PICK_UP(object): Grasp and lift an object. Requires: robot near object, gripper empty
- PLACE(object, surface): Put held object on a surface. Requires: holding object
- PUSH(object, direction): Push an object. Direction: left, right, forward
- OPEN(container): Open a door, drawer, fridge, etc.
- CLOSE(container): Close a door, drawer, fridge, etc.

### Communication
- SAY(message): Speak to nearby humans
- GESTURE(type): Perform gesture. Types: wave, point, nod, shake_head

### Perception
- LOOK_AT(target): Turn head to look at target
- SCAN_ROOM(): Look around to find objects
- IDENTIFY(object): Get information about an object

### Waiting
- WAIT(seconds): Pause execution
- WAIT_FOR(condition): Wait until condition is met

## Constraints
- Can only hold ONE object at a time
- Must be near an object to pick it up
- Cannot walk through closed doors
- Should announce actions to nearby humans for safety
"""
```

### Few-Shot Examples

```python
FEW_SHOT_EXAMPLES = """
## Example Plans

### Example 1: "Bring me a glass of water"
STEP 1: SAY("I'll get you some water")
STEP 2: NAVIGATE(kitchen)
STEP 3: APPROACH(cabinet)
STEP 4: OPEN(cabinet)
STEP 5: PICK_UP(glass)
STEP 6: CLOSE(cabinet)
STEP 7: APPROACH(sink)
STEP 8: FILL_GLASS(glass)
STEP 9: NAVIGATE(living_room)
STEP 10: APPROACH(human)
STEP 11: SAY("Here's your water")
STEP 12: PLACE(glass, coffee_table)

### Example 2: "Answer the door"
STEP 1: SAY("I'll get the door")
STEP 2: NAVIGATE(front_door)
STEP 3: OPEN(front_door)
STEP 4: SAY("Hello, welcome")
STEP 5: GESTURE(wave)
STEP 6: WAIT_FOR(guest_enters)
STEP 7: CLOSE(front_door)

### Example 3: "Clean up the living room"
STEP 1: SAY("I'll tidy up the living room")
STEP 2: SCAN_ROOM()
STEP 3: APPROACH(empty_cup)
STEP 4: PICK_UP(empty_cup)
STEP 5: NAVIGATE(kitchen)
STEP 6: PLACE(empty_cup, sink)
STEP 7: NAVIGATE(living_room)
STEP 8: APPROACH(magazines)
STEP 9: PICK_UP(magazines)
STEP 10: PLACE(magazines, magazine_rack)
STEP 11: SAY("Living room is tidy now")
"""
```

### Dynamic Context Injection

```python
class ContextAwarePlanner(LLMPlanner):
    def __init__(self):
        super().__init__()
        self.scene_graph = None
        self.robot_state = None
    
    def build_context(self):
        """Build context string from current state."""
        context_parts = []
        
        # Robot state
        context_parts.append(f"Robot location: {self.robot_state['location']}")
        context_parts.append(f"Currently holding: {self.robot_state['holding'] or 'nothing'}")
        context_parts.append(f"Battery level: {self.robot_state['battery']}%")
        
        # Scene information
        if self.scene_graph:
            objects = [obj['name'] for obj in self.scene_graph['objects']]
            context_parts.append(f"Visible objects: {', '.join(objects)}")
            
            people = self.scene_graph.get('people', [])
            if people:
                context_parts.append(f"People present: {len(people)} person(s)")
        
        # Recent actions
        if hasattr(self, 'action_history'):
            recent = self.action_history[-3:]
            context_parts.append(f"Recent actions: {recent}")
        
        return "\n".join(context_parts)
    
    def update_state(self, robot_state, scene_graph):
        """Update planner with current state."""
        self.robot_state = robot_state
        self.scene_graph = scene_graph
```

## 25.4 Grounding Language in Actions

### Skill Library

```python
class SkillLibrary:
    """Maps LLM actions to robot skills."""
    
    def __init__(self, robot_interface):
        self.robot = robot_interface
        
        # Action registry
        self.actions = {
            'NAVIGATE': self.navigate,
            'APPROACH': self.approach,
            'PICK_UP': self.pick_up,
            'PLACE': self.place,
            'OPEN': self.open_object,
            'CLOSE': self.close_object,
            'SAY': self.say,
            'GESTURE': self.gesture,
            'LOOK_AT': self.look_at,
            'WAIT': self.wait,
        }
    
    def execute(self, action, parameters):
        """Execute an action with parameters."""
        if action not in self.actions:
            raise ValueError(f"Unknown action: {action}")
        
        return self.actions[action](*parameters)
    
    def navigate(self, location):
        """Navigate to a named location."""
        # Map location name to coordinates
        locations = {
            'kitchen': [2.0, 1.0, 0.0],
            'living_room': [0.0, 0.0, 0.0],
            'bedroom': [-2.0, 3.0, 1.57],
            'front_door': [0.0, -3.0, -1.57],
        }
        
        if location not in locations:
            return False, f"Unknown location: {location}"
        
        coords = locations[location]
        success = self.robot.navigate_to(coords)
        
        return success, f"Navigated to {location}" if success else f"Failed to reach {location}"
    
    def pick_up(self, object_name):
        """Pick up an object."""
        # Find object in scene
        object_pose = self.robot.perception.find_object(object_name)
        
        if object_pose is None:
            return False, f"Cannot find {object_name}"
        
        # Plan grasp
        grasp = self.robot.manipulation.plan_grasp(object_pose)
        
        if grasp is None:
            return False, f"Cannot grasp {object_name}"
        
        # Execute grasp
        success = self.robot.manipulation.execute_grasp(grasp)
        
        return success, f"Picked up {object_name}" if success else f"Failed to pick up {object_name}"
    
    def say(self, message):
        """Speak a message."""
        self.robot.tts.speak(message)
        return True, f"Said: {message}"
    
    def wait(self, seconds):
        """Wait for specified time."""
        import time
        time.sleep(float(seconds))
        return True, f"Waited {seconds} seconds"
```

### Feasibility Checking

```python
class FeasibilityChecker:
    """Check if planned actions are feasible."""
    
    def __init__(self, robot_state, scene_graph):
        self.robot_state = robot_state
        self.scene_graph = scene_graph
    
    def check_plan(self, plan):
        """Check if entire plan is feasible."""
        simulated_state = self.robot_state.copy()
        
        for step in plan:
            feasible, reason = self.check_step(step, simulated_state)
            
            if not feasible:
                return False, step, reason
            
            # Update simulated state
            simulated_state = self.simulate_step(step, simulated_state)
        
        return True, None, None
    
    def check_step(self, step, state):
        """Check if a single step is feasible."""
        action = step['action']
        params = step['parameters']
        
        if action == 'PICK_UP':
            # Check if gripper is empty
            if state['holding'] is not None:
                return False, "Already holding something"
            
            # Check if object exists and is reachable
            obj = params[0]
            if not self.object_exists(obj):
                return False, f"Object {obj} not found"
            
            if not self.is_reachable(obj, state['location']):
                return False, f"Object {obj} is not reachable from current location"
        
        elif action == 'PLACE':
            # Check if holding the object
            obj = params[0]
            if state['holding'] != obj:
                return False, f"Not holding {obj}"
        
        elif action == 'NAVIGATE':
            # Check if location exists
            location = params[0]
            if not self.location_exists(location):
                return False, f"Unknown location: {location}"
        
        return True, None
    
    def simulate_step(self, step, state):
        """Simulate effect of step on state."""
        new_state = state.copy()
        action = step['action']
        params = step['parameters']
        
        if action == 'PICK_UP':
            new_state['holding'] = params[0]
        elif action == 'PLACE':
            new_state['holding'] = None
        elif action == 'NAVIGATE':
            new_state['location'] = params[0]
        
        return new_state
```

## 25.5 Plan Execution and Monitoring

### Plan Executor

```python
class PlanExecutor:
    def __init__(self, skill_library, feasibility_checker, llm_planner):
        self.skills = skill_library
        self.checker = feasibility_checker
        self.planner = llm_planner
        
        self.current_plan = None
        self.current_step = 0
    
    def execute_plan(self, plan):
        """Execute a plan with monitoring and replanning."""
        self.current_plan = plan
        self.current_step = 0
        
        while self.current_step < len(plan):
            step = plan[self.current_step]
            
            # Check feasibility before execution
            feasible, reason = self.checker.check_step(
                step, 
                self.skills.robot.get_state()
            )
            
            if not feasible:
                # Attempt replanning
                success = self.replan(reason)
                if not success:
                    return False, f"Plan failed at step {self.current_step}: {reason}"
                continue
            
            # Execute step
            action = step['action']
            params = step['parameters']
            
            success, message = self.skills.execute(action, params)
            
            if not success:
                # Attempt recovery
                recovered = self.recover_from_failure(step, message)
                if not recovered:
                    return False, f"Step {self.current_step} failed: {message}"
                continue
            
            self.current_step += 1
        
        return True, "Plan completed successfully"
    
    def replan(self, failure_reason):
        """Generate new plan after failure."""
        # Get remaining task
        remaining_steps = self.current_plan[self.current_step:]
        original_task = self.extract_task_goal(remaining_steps)
        
        # Build context with failure information
        context = self.skills.robot.get_state_description()
        context += f"\n\nPrevious plan failed because: {failure_reason}"
        context += "\nPlease generate an alternative plan."
        
        # Generate new plan
        new_plan = self.planner.generate_plan(original_task, context)
        
        if new_plan:
            self.current_plan = new_plan
            self.current_step = 0
            return True
        
        return False
    
    def recover_from_failure(self, failed_step, error_message):
        """Attempt to recover from step failure."""
        action = failed_step['action']
        
        # Action-specific recovery
        if action == 'PICK_UP':
            # Try repositioning and retry
            self.skills.execute('APPROACH', failed_step['parameters'])
            return True  # Will retry the pick
        
        elif action == 'NAVIGATE':
            # Try alternative path
            return self.try_alternative_navigation(failed_step['parameters'][0])
        
        return False
```

## 25.6 Advanced: Chain-of-Thought Planning

### Reasoning with LLMs

```python
class ChainOfThoughtPlanner(LLMPlanner):
    def __init__(self):
        super().__init__()
        
        self.reasoning_prompt = """
Before generating the plan, think through the task step by step:

1. What is the goal?
2. What is the current state?
3. What obstacles or challenges exist?
4. What is the logical sequence of actions?
5. Are there any safety considerations?

Show your reasoning, then provide the plan.
"""
    
    def generate_plan_with_reasoning(self, task, context):
        """Generate plan with explicit reasoning."""
        messages = [
            {"role": "system", "content": self.system_prompt},
            {"role": "user", "content": self.reasoning_prompt},
            {"role": "user", "content": f"Context:\n{context}\n\nTask: {task}"}
        ]
        
        response = self.client.chat.completions.create(
            model=self.model,
            messages=messages,
            temperature=0.3,
            max_tokens=1000
        )
        
        full_response = response.choices[0].message.content
        
        # Extract reasoning and plan
        reasoning = self.extract_reasoning(full_response)
        plan = self.parse_plan(full_response)
        
        return {
            'reasoning': reasoning,
            'plan': plan
        }
```

## 25.7 Summary

In this chapter, you learned:

- **LLM integration**: Using GPT-4 for robot planning
- **Prompt engineering**: Describing capabilities and context
- **Action grounding**: Mapping language to robot skills
- **Feasibility checking**: Validating plans before execution
- **Plan execution**: Monitoring and replanning
- **Chain-of-thought**: Explicit reasoning for better plans

LLMs enable robots to understand complex tasks and generate flexible plans. In the next chapter, we'll explore conversational robotics.

## Review Questions

1. How do LLMs complement traditional robot planning?
2. What should be included in a robot capability prompt?
3. Why is action grounding important?
4. How do you handle plan failures?
5. What is chain-of-thought planning?

## Hands-On Exercise

1. Set up OpenAI API integration
2. Design prompts for your robot's capabilities
3. Implement a skill library with basic actions
4. Create a feasibility checker
5. Build a plan executor with replanning
6. Test with various household tasks
