---
title: Multi-Modal Interaction
description: Integrating vision, language, and action for natural human-robot interaction
sidebar_position: 7
---

# Multi-Modal Interaction

## Introduction to Multi-Modal Systems

Multi-modal interaction in humanoid robotics combines multiple sensory modalities—vision, language, touch, and action—to create more natural and intuitive human-robot interactions. This integration allows robots to understand complex commands, perceive their environment, and execute appropriate responses in a coordinated manner.

## Architecture for Multi-Modal Integration

A multi-modal system typically consists of several interconnected components:

1. **Perception Module**: Processes visual, auditory, and tactile inputs
2. **Language Understanding Module**: Interprets natural language commands
3. **Action Planning Module**: Generates sequences of actions
4. **Execution Module**: Controls robot hardware to execute actions
5. **Integration Layer**: Coordinates between all modules

```python
import numpy as np
import json
from typing import Dict, List, Any
from dataclasses import dataclass

@dataclass
class MultiModalInput:
    """Container for multi-modal input data"""
    text_command: str = ""
    visual_objects: List[Dict] = None
    audio_data: Any = None
    tactile_data: Dict = None
    spatial_context: Dict = None

@dataclass
class MultiModalOutput:
    """Container for multi-modal output data"""
    actions: List[Dict] = None
    speech_response: str = ""
    visual_feedback: Dict = None
    confidence: float = 0.0

class MultiModalFusion:
    """Fuses information from multiple modalities"""
    
    def __init__(self):
        self.confidence_threshold = 0.7
    
    def fuse_inputs(self, multimodal_input: MultiModalInput) -> Dict[str, Any]:
        """Fuse information from different modalities"""
        fused_data = {
            'entities': self.extract_entities(multimodal_input),
            'spatial_relations': self.extract_spatial_relations(multimodal_input),
            'intent': self.extract_intent(multimodal_input),
            'context': self.extract_context(multimodal_input)
        }
        
        return fused_data
    
    def extract_entities(self, multimodal_input: MultiModalInput) -> List[Dict]:
        """Extract entities from text and visual input"""
        entities = []
        
        # Extract entities from text using simple keyword matching
        # In practice, you'd use NLP models
        text_entities = self.extract_text_entities(multimodal_input.text_command)
        entities.extend(text_entities)
        
        # Add visual objects as entities
        if multimodal_input.visual_objects:
            for obj in multimodal_input.visual_objects:
                # Check if this object was mentioned in the text
                obj_name = obj.get('name', '').lower()
                if any(obj_name in entity.get('name', '').lower() for entity in text_entities):
                    # Merge or update entity with visual information
                    for entity in entities:
                        if obj_name in entity.get('name', '').lower():
                            entity['visual_info'] = obj
                            break
                else:
                    entities.append({
                        'name': obj_name,
                        'type': 'object',
                        'visual_info': obj,
                        'confidence': obj.get('confidence', 0.9)
                    })
        
        return entities
    
    def extract_text_entities(self, text: str) -> List[Dict]:
        """Extract entities from text command"""
        # Simple keyword-based entity extraction
        # In practice, use NER models
        entities = []
        
        # Common object categories
        objects = ['ball', 'box', 'cup', 'book', 'bottle', 'apple', 'chair', 'table']
        
        for obj in objects:
            if obj in text.lower():
                entities.append({
                    'name': obj,
                    'type': 'object',
                    'confidence': 0.8
                })
        
        # Common locations
        locations = ['kitchen', 'bedroom', 'living room', 'office', 'table', 'shelf', 'box']
        for loc in locations:
            if loc in text.lower():
                entities.append({
                    'name': loc,
                    'type': 'location',
                    'confidence': 0.8
                })
        
        return entities
    
    def extract_spatial_relations(self, multimodal_input: MultiModalInput) -> List[Dict]:
        """Extract spatial relationships between objects"""
        relations = []
        
        # From text command
        text_relations = self.extract_text_spatial_relations(multimodal_input.text_command)
        relations.extend(text_relations)
        
        # From visual input
        if multimodal_input.visual_objects and multimodal_input.spatial_context:
            visual_relations = self.extract_visual_spatial_relations(
                multimodal_input.visual_objects,
                multimodal_input.spatial_context
            )
            relations.extend(visual_relations)
        
        return relations
    
    def extract_text_spatial_relations(self, text: str) -> List[Dict]:
        """Extract spatial relations from text"""
        relations = []
        
        # Look for spatial prepositions
        spatial_keywords = [
            ('left of', 'left_of'),
            ('right of', 'right_of'),
            ('next to', 'adjacent_to'),
            ('behind', 'behind'),
            ('in front of', 'in_front_of'),
            ('on top of', 'on_top_of'),
            ('under', 'under'),
            ('inside', 'inside'),
            ('outside', 'outside')
        ]
        
        for keyword, relation_type in spatial_keywords:
            if keyword in text.lower():
                # Extract the objects involved in the relation
                parts = text.lower().split(keyword)
                if len(parts) >= 2:
                    relations.append({
                        'type': relation_type,
                        'reference_object': parts[0].strip(),
                        'target_object': parts[1].strip(),
                        'confidence': 0.8
                    })
        
        return relations
    
    def extract_visual_spatial_relations(self, objects: List[Dict], spatial_context: Dict) -> List[Dict]:
        """Extract spatial relations from visual input"""
        relations = []
        
        # Calculate spatial relationships based on object positions
        for i, obj1 in enumerate(objects):
            for j, obj2 in enumerate(objects):
                if i != j:
                    pos1 = np.array(obj1.get('position', [0, 0, 0]))
                    pos2 = np.array(obj2.get('position', [0, 0, 0]))
                    
                    # Calculate distance
                    distance = np.linalg.norm(pos1 - pos2)
                    
                    # Determine spatial relationship based on positions
                    if distance < 0.3:  # Objects are close
                        relations.append({
                            'type': 'adjacent_to',
                            'object1': obj1['name'],
                            'object2': obj2['name'],
                            'distance': distance,
                            'confidence': 0.9
                        })
                    
                    # Check if one is above the other
                    if abs(pos1[2] - pos2[2]) < 0.1 and pos1[2] > pos2[2] + 0.1:
                        relations.append({
                            'type': 'on_top_of',
                            'object_on_top': obj1['name'],
                            'object_below': obj2['name'],
                            'confidence': 0.9
                        })
        
        return relations
    
    def extract_intent(self, multimodal_input: MultiModalInput) -> Dict:
        """Extract the user's intent from the multi-modal input"""
        # Analyze text command for action verbs
        action_verbs = {
            'pick': 'grasp_object',
            'grasp': 'grasp_object', 
            'take': 'grasp_object',
            'move': 'move_object',
            'place': 'place_object',
            'put': 'place_object',
            'go': 'navigate',
            'walk': 'navigate',
            'move_to': 'navigate',
            'bring': 'fetch_object',
            'give': 'deliver_object',
            'show': 'point_to_object',
            'look': 'look_at_object'
        }
        
        text_lower = multimodal_input.text_command.lower()
        
        for verb, action in action_verbs.items():
            if verb in text_lower:
                return {
                    'action': action,
                    'confidence': 0.85,
                    'parameters': self.extract_action_parameters(multimodal_input, action)
                }
        
        # Default intent if no specific action found
        return {
            'action': 'understand_command',
            'confidence': 0.7,
            'parameters': {}
        }
    
    def extract_action_parameters(self, multimodal_input: MultiModalInput, action: str) -> Dict:
        """Extract parameters for the specified action"""
        params = {}
        
        # Extract object to act upon
        entities = self.extract_entities(multimodal_input)
        objects = [e for e in entities if e['type'] == 'object']
        
        if objects:
            params['target_object'] = objects[0]['name']
            if 'visual_info' in objects[0]:
                params['object_pose'] = objects[0]['visual_info'].get('position', [0, 0, 0])
        
        # Extract destination if applicable
        if action in ['place_object', 'navigate', 'move_object']:
            locations = [e for e in entities if e['type'] == 'location']
            if locations:
                params['destination'] = locations[0]['name']
        
        return params
    
    def extract_context(self, multimodal_input: MultiModalInput) -> Dict:
        """Extract contextual information"""
        return {
            'environment': multimodal_input.spatial_context.get('environment', 'unknown'),
            'time': multimodal_input.spatial_context.get('timestamp', None),
            'user_intent_confidence': self.estimate_intent_confidence(multimodal_input)
        }
    
    def estimate_intent_confidence(self, multimodal_input: MultiModalInput) -> float:
        """Estimate confidence in the interpreted intent"""
        # Simple confidence estimation based on input consistency
        text_confidence = 0.8 if multimodal_input.text_command.strip() else 0.0
        visual_confidence = 0.7 if multimodal_input.visual_objects else 0.0
        
        # Higher confidence if both modalities agree
        if text_confidence > 0 and visual_confidence > 0:
            # Check if detected objects match text command
            text_entities = self.extract_text_entities(multimodal_input.text_command)
            visual_entities = [obj['name'] for obj in multimodal_input.visual_objects or []]
            
            if any(ent['name'] in visual_entities for ent in text_entities):
                return min(1.0, text_confidence + visual_confidence)
        
        return max(text_confidence, visual_confidence)
```

## Perception Integration

The perception module processes inputs from multiple sensors:

```python
class PerceptionModule:
    """Processes visual, auditory, and tactile inputs"""
    
    def __init__(self):
        self.object_detector = None  # Initialize with actual object detection model
        self.speech_recognizer = None  # Initialize with actual ASR system
        self.tactile_sensor_processor = None
    
    def process_visual_input(self, image_data) -> List[Dict]:
        """Process visual input to detect objects and their properties"""
        # This would interface with a computer vision model
        # For demonstration, returning simulated data
        return [
            {
                'name': 'red_ball',
                'position': [1.2, 0.5, 0.1],
                'confidence': 0.92,
                'bbox': [100, 150, 200, 250],
                'color': 'red'
            },
            {
                'name': 'blue_box',
                'position': [0.8, -0.3, 0.2],
                'confidence': 0.88,
                'bbox': [300, 100, 400, 200],
                'color': 'blue'
            }
        ]
    
    def process_auditory_input(self, audio_data) -> str:
        """Process auditory input to extract speech"""
        # This would interface with a speech recognition system
        # For demonstration, returning simulated text
        return "Please pick up the red ball and place it in the blue box"
    
    def process_tactile_input(self, tactile_data) -> Dict:
        """Process tactile input to detect contact and forces"""
        # This would interface with tactile sensors
        # For demonstration, returning simulated data
        return {
            'contact_detected': True,
            'force': [2.5, 1.2, 0.8],
            'contact_location': 'right_gripper'
        }
    
    def process_spatial_context(self, pose_data) -> Dict:
        """Process spatial context information"""
        return {
            'robot_position': pose_data.get('position', [0, 0, 0]),
            'environment': 'indoor',
            'timestamp': pose_data.get('timestamp', 0)
        }
```

## Language Understanding Module

The language understanding module interprets natural language commands:

```python
class LanguageUnderstandingModule:
    """Interprets natural language commands"""
    
    def __init__(self):
        self.semantic_parser = None  # Initialize with NLP model
        self.knowledge_base = {}     # Store information about objects and environment
    
    def parse_command(self, text_command: str, entities: List[Dict], relations: List[Dict]) -> Dict:
        """Parse natural language command into structured action"""
        # This would use NLP techniques to understand the command
        # For demonstration, using simple pattern matching
        
        command_lower = text_command.lower()
        
        # Determine action type
        action = self.determine_action_type(command_lower)
        
        # Extract target objects
        targets = self.extract_targets(command_lower, entities)
        
        # Extract destination if applicable
        destination = self.extract_destination(command_lower, entities)
        
        # Extract spatial constraints
        spatial_constraints = self.extract_spatial_constraints(command_lower, relations)
        
        return {
            'action': action,
            'targets': targets,
            'destination': destination,
            'spatial_constraints': spatial_constraints,
            'command': text_command,
            'confidence': self.estimate_parsing_confidence(text_command)
        }
    
    def determine_action_type(self, command: str) -> str:
        """Determine the type of action from the command"""
        action_keywords = {
            'grasp': ['pick', 'grasp', 'take', 'grab', 'hold'],
            'navigate': ['go', 'walk', 'move to', 'go to', 'navigate to'],
            'place': ['place', 'put', 'set', 'drop', 'release'],
            'transport': ['bring', 'carry', 'move', 'deliver', 'give'],
            'point': ['point', 'show', 'indicate', 'look at'],
            'describe': ['what is', 'describe', 'tell me about']
        }
        
        for action, keywords in action_keywords.items():
            if any(keyword in command for keyword in keywords):
                return action
        
        return 'unknown'
    
    def extract_targets(self, command: str, entities: List[Dict]) -> List[Dict]:
        """Extract target objects from the command"""
        targets = []
        
        # Look for object entities mentioned in the command
        for entity in entities:
            if entity['name'].lower() in command and entity['type'] == 'object':
                targets.append(entity)
        
        return targets
    
    def extract_destination(self, command: str, entities: List[Dict]) -> Dict:
        """Extract destination from the command"""
        location_keywords = ['to', 'in', 'on', 'at', 'into', 'onto']
        
        for entity in entities:
            if entity['name'].lower() in command and entity['type'] == 'location':
                return entity
        
        return None
    
    def extract_spatial_constraints(self, command: str, relations: List[Dict]) -> List[Dict]:
        """Extract spatial constraints from the command"""
        constraints = []
        
        # Look for spatial relations in the command
        for relation in relations:
            if relation['type'] in command:
                constraints.append(relation)
        
        return constraints
    
    def estimate_parsing_confidence(self, command: str) -> float:
        """Estimate confidence in the command parsing"""
        # Simple confidence estimation based on command structure
        if len(command.split()) < 2:
            return 0.3  # Short commands have lower confidence
        
        # Check for key action words
        action_words = ['pick', 'grasp', 'take', 'go', 'move', 'place', 'put', 'bring', 'give']
        if any(word in command.lower() for word in action_words):
            return 0.85
        else:
            return 0.5
```

## Action Planning and Execution

The action planning module generates executable sequences:

```python
class ActionPlanner:
    """Plans sequences of actions based on parsed commands"""
    
    def __init__(self):
        self.trajectory_generator = None  # For path planning
        self.kinematic_solver = None      # For inverse kinematics
    
    def plan_action_sequence(self, parsed_command: Dict, current_state: Dict) -> List[Dict]:
        """Plan a sequence of actions to fulfill the command"""
        action_sequence = []
        
        action_type = parsed_command['action']
        
        if action_type == 'grasp':
            action_sequence = self.plan_grasp_sequence(parsed_command, current_state)
        elif action_type == 'navigate':
            action_sequence = self.plan_navigation_sequence(parsed_command, current_state)
        elif action_type == 'place':
            action_sequence = self.plan_placement_sequence(parsed_command, current_state)
        elif action_type == 'transport':
            action_sequence = self.plan_transport_sequence(parsed_command, current_state)
        else:
            # Default: just acknowledge the command
            action_sequence = [
                {
                    'action_type': 'speak',
                    'parameters': {'text': f"I understand your command: {parsed_command['command']}"}
                }
            ]
        
        return action_sequence
    
    def plan_grasp_sequence(self, parsed_command: Dict, current_state: Dict) -> List[Dict]:
        """Plan sequence for grasping an object"""
        sequence = []
        
        targets = parsed_command.get('targets', [])
        if not targets:
            return [{'action_type': 'speak', 'parameters': {'text': 'I don\'t see which object to grasp.'}}]
        
        target = targets[0]  # Take the first target
        
        # 1. Navigate to the object if needed
        robot_pos = current_state.get('position', [0, 0, 0])
        obj_pos = target.get('visual_info', {}).get('position', [1, 0, 0])
        
        distance = np.linalg.norm(np.array(robot_pos[:2]) - np.array(obj_pos[:2]))
        if distance > 0.5:  # If further than 50cm, navigate first
            sequence.append({
                'action_type': 'navigate',
                'parameters': {
                    'target_position': [obj_pos[0], obj_pos[1], robot_pos[2]],  # Same Z as robot
                    'approach_vector': self.calculate_approach_vector(obj_pos, robot_pos)
                }
            })
        
        # 2. Plan grasp approach
        sequence.append({
            'action_type': 'approach_object',
            'parameters': {
                'object_pose': obj_pos,
                'approach_distance': 0.1  # 10cm from object
            }
        })
        
        # 3. Execute grasp
        sequence.append({
            'action_type': 'grasp_object',
            'parameters': {
                'object_name': target['name'],
                'object_pose': obj_pos,
                'grasp_type': 'top_grasp'  # Determine based on object properties
            }
        })
        
        # 4. Lift object slightly
        sequence.append({
            'action_type': 'lift_object',
            'parameters': {
                'lift_height': 0.05  # 5cm
            }
        })
        
        return sequence
    
    def calculate_approach_vector(self, obj_pos, robot_pos):
        """Calculate the best approach vector to an object"""
        # Approach from the front for safety
        direction = np.array(obj_pos) - np.array(robot_pos)
        direction[2] = 0  # Keep Z component zero for planar approach
        direction = direction / np.linalg.norm(direction)
        
        # Offset slightly to avoid collision
        approach_point = np.array(obj_pos) - 0.3 * direction  # 30cm away
        
        return approach_point.tolist()
    
    def plan_navigation_sequence(self, parsed_command: Dict, current_state: Dict) -> List[Dict]:
        """Plan sequence for navigation"""
        sequence = []
        
        destination = parsed_command.get('destination')
        if not destination:
            return [{'action_type': 'speak', 'parameters': {'text': 'I don\'t know where to go.'}}]
        
        # Find the position of the destination in the environment
        dest_pos = self.get_destination_position(destination, current_state)
        
        if dest_pos:
            sequence.append({
                'action_type': 'navigate',
                'parameters': {
                    'target_position': dest_pos,
                    'path_planning_mode': 'avoid_obstacles'
                }
            })
        else:
            sequence.append({
                'action_type': 'speak',
                'parameters': {'text': f"I don't know where {destination['name']} is."}
            })
        
        return sequence
    
    def get_destination_position(self, destination, current_state):
        """Get the position of a destination"""
        # This would query a map or localization system
        # For demonstration, returning a fixed position for known locations
        known_locations = {
            'kitchen': [3.0, 2.0, 0.0],
            'bedroom': [-2.0, 1.5, 0.0],
            'living room': [0.0, -2.0, 0.0],
            'office': [2.5, -1.0, 0.0]
        }
        
        return known_locations.get(destination['name'].lower())
    
    def plan_placement_sequence(self, parsed_command: Dict, current_state: Dict) -> List[Dict]:
        """Plan sequence for placing an object"""
        sequence = []
        
        # 1. Check if holding an object
        if not current_state.get('holding_object'):
            sequence.append({
                'action_type': 'speak',
                'parameters': {'text': 'I am not holding any object to place.'}
            })
            return sequence
        
        destination = parsed_command.get('destination')
        if not destination:
            sequence.append({
                'action_type': 'speak',
                'parameters': {'text': 'I don\'t know where to place the object.'}
            })
            return sequence
        
        # 2. Navigate to destination if needed
        robot_pos = current_state.get('position', [0, 0, 0])
        dest_pos = self.get_destination_position(destination, current_state)
        
        if dest_pos:
            distance = np.linalg.norm(np.array(robot_pos[:2]) - np.array(dest_pos[:2]))
            if distance > 0.5:
                sequence.append({
                    'action_type': 'navigate',
                    'parameters': {
                        'target_position': [dest_pos[0], dest_pos[1], robot_pos[2]]
                    }
                })
        
        # 3. Position for placement
        sequence.append({
            'action_type': 'position_for_placement',
            'parameters': {
                'destination': dest_pos,
                'object_to_place': current_state['holding_object']
            }
        })
        
        # 4. Execute placement
        sequence.append({
            'action_type': 'place_object',
            'parameters': {
                'placement_position': dest_pos,
                'placement_orientation': [0, 0, 0, 1]  # Default orientation
            }
        })
        
        # 5. Release object
        sequence.append({
            'action_type': 'release_object',
            'parameters': {}
        })
        
        return sequence
    
    def plan_transport_sequence(self, parsed_command: Dict, current_state: Dict) -> List[Dict]:
        """Plan sequence for transporting an object from one place to another"""
        sequence = []
        
        targets = parsed_command.get('targets', [])
        destination = parsed_command.get('destination')
        
        if not targets:
            sequence.append({
                'action_type': 'speak',
                'parameters': {'text': 'I don\'t know what object to transport.'}
            })
            return sequence
        
        if not destination:
            sequence.append({
                'action_type': 'speak',
                'parameters': {'text': 'I don\'t know where to transport the object.'}
            })
            return sequence
        
        # 1. Grasp the object
        grasp_sequence = self.plan_grasp_sequence(
            {'action': 'grasp', 'targets': targets}, 
            current_state
        )
        sequence.extend(grasp_sequence)
        
        # 2. Navigate to destination
        nav_command = {'action': 'navigate', 'destination': destination}
        nav_sequence = self.plan_navigation_sequence(nav_command, current_state)
        sequence.extend(nav_sequence)
        
        # 3. Place the object
        place_command = {'action': 'place', 'destination': destination}
        place_sequence = self.plan_placement_sequence(place_command, 
                                                     {**current_state, 'holding_object': targets[0]['name']})
        sequence.extend(place_sequence)
        
        return sequence
```

## ROS 2 Multi-Modal Node

Let's implement a ROS 2 node that integrates all the multi-modal components:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from geometry_msgs.msg import Pose, Point, Vector3
from std_msgs.msg import String, Bool, Float64MultiArray
from audio_common_msgs.msg import AudioData
from interactive_msgs.msg import InteractiveObject
import json

class MultiModalInteractionNode(Node):
    def __init__(self):
        super().__init__('multi_modal_interaction_node')
        
        # Subscribers
        self.text_command_sub = self.create_subscription(
            String,
            '/text_commands',
            self.text_command_callback,
            10
        )
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )
        
        self.audio_sub = self.create_subscription(
            AudioData,
            '/audio',
            self.audio_callback,
            10
        )
        
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        self.robot_pose_sub = self.create_subscription(
            Pose,
            '/robot_pose',
            self.pose_callback,
            10
        )
        
        # Publishers
        self.action_sequence_pub = self.create_publisher(String, '/action_sequence', 10)
        self.speech_pub = self.create_publisher(String, '/speech_output', 10)
        self.visualization_pub = self.create_publisher(InteractiveObject, '/visualization', 10)
        
        # Initialize modules
        self.perception_module = PerceptionModule()
        self.language_module = LanguageUnderstandingModule()
        self.fusion_module = MultiModalFusion()
        self.action_planner = ActionPlanner()
        
        # Robot state
        self.current_pose = None
        self.holding_object = None
        self.environment_map = {}  # For storing object locations
        
    def text_command_callback(self, msg):
        """Handle text commands from user"""
        self.get_logger().info(f'Received text command: {msg.data}')
        
        # Process with other modalities to create multi-modal input
        multimodal_input = MultiModalInput(
            text_command=msg.data,
            visual_objects=self.get_recently_seen_objects(),
            spatial_context=self.get_spatial_context()
        )
        
        # Process the multi-modal input
        self.process_multimodal_input(multimodal_input)
    
    def image_callback(self, msg):
        """Process visual input"""
        # In a real system, we would process the image to detect objects
        # For this example, we'll just store the timestamp
        self.last_image_time = self.get_clock().now()
    
    def audio_callback(self, msg):
        """Process audio input"""
        # In a real system, we would convert audio to text
        # For this example, we'll just store the timestamp
        self.last_audio_time = self.get_clock().now()
    
    def laser_callback(self, msg):
        """Process laser scan for obstacle detection"""
        # Process laser data for navigation and safety
        self.last_laser_time = self.get_clock().now()
    
    def pose_callback(self, msg):
        """Update robot pose"""
        self.current_pose = [
            msg.position.x, msg.position.y, msg.position.z,
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        ]
    
    def get_recently_seen_objects(self):
        """Get objects detected in recent visual processing"""
        # In a real system, this would return recently detected objects
        # For this example, returning simulated data
        return [
            {
                'name': 'red_ball',
                'position': [1.2, 0.5, 0.1],
                'confidence': 0.92,
                'bbox': [100, 150, 200, 250],
                'color': 'red'
            },
            {
                'name': 'blue_box',
                'position': [0.8, -0.3, 0.2],
                'confidence': 0.88,
                'bbox': [300, 100, 400, 200],
                'color': 'blue'
            }
        ]
    
    def get_spatial_context(self):
        """Get current spatial context"""
        return {
            'robot_position': self.current_pose or [0, 0, 0],
            'environment': 'indoor_lab',
            'timestamp': self.get_clock().now().nanoseconds
        }
    
    def process_multimodal_input(self, multimodal_input: MultiModalInput):
        """Process multi-modal input and generate response"""
        # Fuse information from different modalities
        fused_data = self.fusion_module.fuse_inputs(multimodal_input)
        
        # Parse the language component
        parsed_command = self.language_module.parse_command(
            multimodal_input.text_command,
            fused_data['entities'],
            fused_data['spatial_relations']
        )
        
        # Check confidence in understanding
        if fused_data['context']['user_intent_confidence'] < self.fusion_module.confidence_threshold:
            # Ask for clarification
            clarification_request = self.generate_clarification_request(multimodal_input, fused_data)
            self.speech_pub.publish(String(data=clarification_request))
            return
        
        # Plan action sequence
        current_state = {
            'position': self.current_pose or [0, 0, 0],
            'holding_object': self.holding_object
        }
        
        action_sequence = self.action_planner.plan_action_sequence(parsed_command, current_state)
        
        # Publish action sequence
        action_msg = String()
        action_msg.data = json.dumps(action_sequence)
        self.action_sequence_pub.publish(action_msg)
        
        # Provide feedback to user
        feedback = self.generate_feedback(parsed_command, action_sequence)
        self.speech_pub.publish(String(data=feedback))
        
        self.get_logger().info(f'Planned action sequence: {action_sequence}')
    
    def generate_clarification_request(self, multimodal_input: MultiModalInput, fused_data: Dict) -> str:
        """Generate a request for clarification when confidence is low"""
        if not fused_data['entities']:
            return "I didn't understand what you want me to do. Could you please repeat your command?"
        
        # Find entities with low confidence
        low_confidence_entities = [e for e in fused_data['entities'] if e.get('confidence', 1.0) < 0.7]
        
        if low_confidence_entities:
            entity_names = [e['name'] for e in low_confidence_entities]
            return f"I'm not sure I correctly identified: {', '.join(entity_names)}. Could you clarify?"
        
        return "I didn't understand your command. Could you please repeat it?"
    
    def generate_feedback(self, parsed_command: Dict, action_sequence: List[Dict]) -> str:
        """Generate verbal feedback about the planned actions"""
        if not action_sequence:
            return "I cannot perform that action."
        
        action_type = parsed_command['action']
        
        if action_type == 'grasp':
            targets = parsed_command.get('targets', [])
            if targets:
                return f"I will pick up the {targets[0]['name']}."
        
        elif action_type == 'navigate':
            destination = parsed_command.get('destination')
            if destination:
                return f"I will go to the {destination['name']}."
        
        elif action_type == 'transport':
            targets = parsed_command.get('targets', [])
            destination = parsed_command.get('destination')
            if targets and destination:
                return f"I will take the {targets[0]['name']} to the {destination['name']}."
        
        return "I will carry out the requested action."

def main(args=None):
    rclpy.init(args=args)
    node = MultiModalInteractionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Context Awareness and Memory

For more sophisticated interaction, robots need to maintain context and memory:

```python
class ContextManager:
    """Manages context and memory for multi-modal interaction"""
    
    def __init__(self):
        self.episodic_memory = []  # Stores past interactions
        self.semantic_memory = {}  # Stores learned facts about the world
        self.working_memory = {}   # Stores current interaction context
    
    def update_context(self, multimodal_input: MultiModalInput, multimodal_output: MultiModalOutput):
        """Update context based on input and output"""
        # Store the interaction in episodic memory
        interaction = {
            'timestamp': self.get_current_time(),
            'input': multimodal_input,
            'output': multimodal_output,
            'outcome': 'success'  # This would be updated based on execution feedback
        }
        
        self.episodic_memory.append(interaction)
        
        # Update semantic memory with new facts
        self.update_semantic_memory(multimodal_input, multimodal_output)
        
        # Update working memory for current interaction
        self.update_working_memory(multimodal_input, multimodal_output)
    
    def update_semantic_memory(self, input_data: MultiModalInput, output_data: MultiModalOutput):
        """Update long-term knowledge based on interaction"""
        # Extract facts from the interaction
        if input_data.visual_objects:
            for obj in input_data.visual_objects:
                obj_name = obj['name']
                if obj_name not in self.semantic_memory:
                    self.semantic_memory[obj_name] = {}
                
                # Update object properties
                self.semantic_memory[obj_name].update({
                    'position': obj.get('position'),
                    'color': obj.get('color'),
                    'shape': obj.get('shape', 'unknown'),
                    'last_seen': self.get_current_time()
                })
    
    def update_working_memory(self, input_data: MultiModalInput, output_data: MultiModalOutput):
        """Update short-term context for current interaction"""
        # Track current task
        if output_data.actions:
            self.working_memory['current_task'] = output_data.actions[0].get('action_type', 'unknown')
        
        # Track what the robot is holding
        if output_data.actions:
            for action in output_data.actions:
                if action.get('action_type') == 'grasp_object':
                    self.working_memory['holding'] = action.get('parameters', {}).get('object_name')
                elif action.get('action_type') == 'release_object':
                    self.working_memory['holding'] = None
    
    def get_current_context(self) -> Dict:
        """Get the current interaction context"""
        return {
            'working_memory': self.working_memory.copy(),
            'recent_interactions': self.episodic_memory[-5:],  # Last 5 interactions
            'known_objects': self.semantic_memory
        }
    
    def get_current_time(self):
        """Get current time (in a real system, this would interface with ROS time)"""
        import time
        return time.time()
```

## Conclusion

Multi-modal interaction enables humanoid robots to engage in natural, human-like communication by integrating information from multiple sensory channels. By combining perception, language understanding, and action planning, we create robots that can understand complex commands, perceive their environment, and respond appropriately. This forms the foundation for the capstone project where we'll integrate all these components into a fully autonomous humanoid system.