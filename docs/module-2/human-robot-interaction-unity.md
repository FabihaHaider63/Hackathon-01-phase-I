---
title: Human–Robot Interaction in Unity
description: Creating intuitive interfaces and interaction systems for humans to work with humanoid robots in Unity-based digital twins
sidebar_position: 14
---

# Human–Robot Interaction in Unity

## Overview

Human–Robot Interaction (HRI) in Unity-based digital twins provides intuitive interfaces for humans to monitor, control, and collaborate with humanoid robots. This chapter explores how to design and implement effective interaction systems that bridge the gap between human operators and robotic systems, making complex robotic behaviors accessible and understandable.

## Designing Intuitive Control Interfaces

### Interface Design Principles

Human-robot interaction interfaces should follow these principles:

1. **Clarity**: Make robot status and capabilities clearly visible
2. **Directness**: Allow users to directly manipulate robot behaviors
3. **Feedback**: Provide immediate visual and haptic feedback
4. **Safety**: Include safeguards to prevent unsafe robot actions
5. **Accessibility**: Design for users with varying levels of technical expertise

### 3D Interface Elements

Unity's 3D UI capabilities allow for intuitive spatial interfaces:

```csharp
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class RobotUIController : MonoBehaviour
{
    [Header("UI Elements")]
    [SerializeField] private GameObject controlPanel;
    [SerializeField] private TextMeshProUGUI statusText;
    [SerializeField] private Slider velocitySlider;
    [SerializeField] private Button[] actionButtons;
    
    [Header("Robot References")]
    [SerializeField] private GameObject robot;
    [SerializeField] private ROS2UnityBridge rosBridge;
    
    void Start()
    {
        SetupUI();
        UpdateStatusDisplay();
    }
    
    void SetupUI()
    {
        // Initialize velocity slider
        velocitySlider.minValue = 0f;
        velocitySlider.maxValue = 1f;
        velocitySlider.value = 0.5f;
        
        // Setup action buttons
        for (int i = 0; i < actionButtons.Length; i++)
        {
            int buttonIndex = i; // Capture for closure
            actionButtons[i].onClick.AddListener(() => ExecuteRobotAction(buttonIndex));
        }
    }
    
    void UpdateStatusDisplay()
    {
        // Update status based on robot data
        statusText.text = "Robot Status: Ready";
    }
    
    void ExecuteRobotAction(int actionIndex)
    {
        // Send command to robot based on button pressed
        switch (actionIndex)
        {
            case 0: 
                rosBridge.SendCommand("move_forward", velocitySlider.value);
                break;
            case 1: 
                rosBridge.SendCommand("turn_left", velocitySlider.value);
                break;
            case 2: 
                rosBridge.SendCommand("grasp_object", velocitySlider.value);
                break;
            // Additional actions...
        }
    }
}
```

## Visualization of Robot Intent and State

### State Visualization

Visualizing robot internal states helps humans understand robot behavior:

- **Intent indicators**: Show where the robot plans to move
- **Attention visualization**: Show where the robot is looking or focusing
- **Mode indicators**: Show current operational mode (navigation, manipulation, etc.)
- **Safety status**: Show safety system status and constraints

```csharp
using UnityEngine;

public class RobotStateVisualizer : MonoBehaviour
{
    [Header("Visualization Objects")]
    [SerializeField] private GameObject intentArrow;
    [SerializeField] private GameObject attentionIndicator;
    [SerializeField] private Material[] stateMaterials;
    [SerializeField] private Renderer robotRenderer;
    
    [Header("Robot Data")]
    [SerializeField] private RobotData robotData;
    
    void Update()
    {
        UpdateIntentVisualization();
        UpdateAttentionVisualization();
        UpdateStateVisualization();
    }
    
    void UpdateIntentVisualization()
    {
        if (robotData.currentTarget != null)
        {
            intentArrow.SetActive(true);
            Vector3 direction = (robotData.currentTarget.position - transform.position).normalized;
            intentArrow.transform.position = transform.position + Vector3.up * 0.5f;
            intentArrow.transform.rotation = Quaternion.LookRotation(direction);
        }
        else
        {
            intentArrow.SetActive(false);
        }
    }
    
    void UpdateAttentionVisualization()
    {
        if (robotData.attentionTarget != null)
        {
            attentionIndicator.SetActive(true);
            attentionIndicator.transform.LookAt(robotData.attentionTarget);
        }
        else
        {
            attentionIndicator.SetActive(false);
        }
    }
    
    void UpdateStateVisualization()
    {
        // Change robot material based on current state
        int stateIndex = (int)robotData.currentState;
        if (stateIndex < stateMaterials.Length)
        {
            robotRenderer.material = stateMaterials[stateIndex];
        }
    }
}

[System.Serializable]
public class RobotData
{
    public Transform currentTarget;
    public Transform attentionTarget;
    public RobotState currentState;
}

public enum RobotState
{
    Idle,
    Moving,
    Manipulating,
    Charging,
    Error
}
```

### Path and Trajectory Visualization

Showing planned paths helps humans anticipate robot movements:

- Projected path visualization
- Dynamic obstacle avoidance indicators
- Alternative path suggestions

## Direct Robot Control Methods

### Teleoperation Interfaces

Direct control of robot movements:

1. **Point-and-click navigation**: Users click on locations to move the robot
2. **Virtual joysticks**: For more precise control
3. **Gesture-based controls**: Using mouse or touch movements
4. **Keyboard controls**: For quick maneuvering

```csharp
using UnityEngine;

public class RobotTeleoperation : MonoBehaviour
{
    [Header("Control References")]
    [SerializeField] private Camera mainCamera;
    [SerializeField] private GameObject robot;
    [SerializeField] private ROS2UnityBridge rosBridge;
    
    [Header("Control Parameters")]
    [SerializeField] private float moveSpeed = 2.0f;
    [SerializeField] private float turnSpeed = 100.0f;
    
    void Update()
    {
        HandlePointAndClickNavigation();
        HandleKeyboardControl();
    }
    
    void HandlePointAndClickNavigation()
    {
        if (Input.GetMouseButtonDown(0))
        {
            Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);
            RaycastHit hit;
            
            if (Physics.Raycast(ray, out hit))
            {
                // Send navigation command to robot
                Vector3 targetPosition = new Vector3(hit.point.x, robot.transform.position.y, hit.point.z);
                rosBridge.SendNavigationCommand(targetPosition);
                
                // Visualize planned path
                VisualizePath(robot.transform.position, targetPosition);
            }
        }
    }
    
    void HandleKeyboardControl()
    {
        float horizontal = Input.GetAxis("Horizontal"); // A/D or arrow keys
        float vertical = Input.GetAxis("Vertical");     // W/S or arrow keys
        
        if (Mathf.Abs(horizontal) > 0.1f || Mathf.Abs(vertical) > 0.1f)
        {
            // Send velocity command to robot
            Vector3 movement = new Vector3(horizontal, 0, vertical) * moveSpeed * Time.deltaTime;
            rosBridge.SendVelocityCommand(movement, Vector3.zero);
        }
    }
    
    void VisualizePath(Vector3 start, Vector3 end)
    {
        // Create temporary path visualization
        GameObject path = new GameObject("PathVisualization");
        LineRenderer lineRenderer = path.AddComponent<LineRenderer>();
        lineRenderer.positionCount = 2;
        lineRenderer.SetPosition(0, start);
        lineRenderer.SetPosition(1, end);
        lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        lineRenderer.startWidth = 0.1f;
        lineRenderer.endWidth = 0.1f;
        lineRenderer.startColor = Color.blue;
        lineRenderer.endColor = Color.cyan;
        
        // Destroy after 5 seconds
        Destroy(path, 5f);
    }
}
```

### Voice Command Integration

Voice interfaces can make robot control more natural:

- Integrate with speech recognition services
- Create voice command vocabularies
- Provide voice feedback on robot status

## Safety and Verification Systems

### Safety Overlay Interface

Critical safety information should always be visible:

```csharp
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class SafetyOverlay : MonoBehaviour
{
    [Header("Safety UI Elements")]
    [SerializeField] private TextMeshProUGUI safetyStatusText;
    [SerializeField] private Image safetyIndicator;
    [SerializeField] private Button emergencyStopButton;
    [SerializeField] private Slider safetyMarginSlider;
    
    [Header("Safety Data")]
    [SerializeField] private SafetySystem safetySystem;
    
    void Start()
    {
        emergencyStopButton.onClick.AddListener(TriggerEmergencyStop);
        safetyMarginSlider.onValueChanged.AddListener(OnSafetyMarginChanged);
        UpdateSafetyDisplay();
    }
    
    void Update()
    {
        UpdateSafetyDisplay();
    }
    
    void UpdateSafetyDisplay()
    {
        if (safetySystem.IsSafe())
        {
            safetyIndicator.color = Color.green;
            safetyStatusText.text = "SAFE";
        }
        else
        {
            safetyIndicator.color = Color.red;
            safetyStatusText.text = "WARNING: Safety violation detected!";
        }
    }
    
    void TriggerEmergencyStop()
    {
        safetySystem.TriggerEmergencyStop();
        rosBridge.SendEmergencyStop();
    }
    
    void OnSafetyMarginChanged(float value)
    {
        safetySystem.SetSafetyMargin(value);
    }
}
```

### Verification and Confirmation

For critical operations, implement verification steps:

- Confirmation dialogs for dangerous actions
- Pre-execution visualization of planned actions
- Reversible operations where possible

## Multi-Modal Interaction

### Visual, Auditory, and Haptic Feedback

Combining different feedback modalities enhances interaction:

- **Visual**: Status indicators, animations, HUD elements
- **Auditory**: Status sounds, warnings, speech output
- **Haptic**: Force feedback for teleoperation

### Example: Multi-Modal Feedback System

```csharp
using UnityEngine;

public class MultiModalFeedback : MonoBehaviour
{
    [Header("Feedback Systems")]
    [SerializeField] private AudioSource audioSource;
    [SerializeField] private Light statusLight;
    [SerializeField] private Animation robotAnimation;
    
    [Header("Feedback Assets")]
    [SerializeField] private AudioClip[] statusSounds;
    [SerializeField] private Color[] statusColors;
    [SerializeField] private string[] statusAnimations;
    
    public void ProvideFeedback(RobotFeedbackType feedbackType)
    {
        switch (feedbackType)
        {
            case RobotFeedbackType.Success:
                PlayAudio(0); // Success sound
                SetLightColor(0); // Green
                PlayAnimation(0); // Success animation
                break;
                
            case RobotFeedbackType.Warning:
                PlayAudio(1); // Warning sound
                SetLightColor(1); // Yellow
                PlayAnimation(1); // Warning animation
                break;
                
            case RobotFeedbackType.Error:
                PlayAudio(2); // Error sound
                SetLightColor(2); // Red
                PlayAnimation(2); // Error animation
                break;
        }
    }
    
    void PlayAudio(int soundIndex)
    {
        if (soundIndex < statusSounds.Length)
        {
            audioSource.clip = statusSounds[soundIndex];
            audioSource.Play();
        }
    }
    
    void SetLightColor(int colorIndex)
    {
        if (colorIndex < statusColors.Length)
        {
            statusLight.color = statusColors[colorIndex];
        }
    }
    
    void PlayAnimation(int animIndex)
    {
        if (animIndex < statusAnimations.Length)
        {
            robotAnimation.Play(statusAnimations[animIndex]);
        }
    }
}

public enum RobotFeedbackType
{
    Success,
    Warning,
    Error
}
```

## Collaboration and Task Delegation

### Task-Based Interfaces

Allow humans to delegate tasks rather than direct control:

- Task specification through GUI
- Progress visualization
- Interruption and modification capabilities

### Example: Task Delegation Interface

```csharp
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.Collections.Generic;

public class TaskDelegationInterface : MonoBehaviour
{
    [Header("UI Elements")]
    [SerializeField] private TMP_Dropdown taskDropdown;
    [SerializeField] private Button assignTaskButton;
    [SerializeField] private TextMeshProUGUI taskStatusText;
    [SerializeField] private Slider taskPrioritySlider;
    
    [Header("Robot Reference")]
    [SerializeField] private ROS2UnityBridge rosBridge;
    
    [Header("Available Tasks")]
    private List<string> availableTasks = new List<string>
    {
        "Fetch Object",
        "Navigate to Location",
        "Inspect Area",
        "Transport Item",
        "Monitor Station"
    };
    
    private string currentTask = "";
    private bool taskAssigned = false;
    
    void Start()
    {
        SetupTaskDropdown();
        assignTaskButton.onClick.AddListener(AssignTask);
    }
    
    void SetupTaskDropdown()
    {
        taskDropdown.ClearOptions();
        taskDropdown.AddOptions(availableTasks);
        taskDropdown.onValueChanged.AddListener(OnTaskSelected);
    }
    
    void OnTaskSelected(int value)
    {
        currentTask = availableTasks[value];
    }
    
    void AssignTask()
    {
        if (!string.IsNullOrEmpty(currentTask))
        {
            float priority = taskPrioritySlider.value;
            
            // Send task assignment to robot
            rosBridge.AssignTask(currentTask, priority);
            
            taskAssigned = true;
            UpdateTaskStatus($"Task '{currentTask}' assigned with priority {priority:F2}");
        }
    }
    
    void UpdateTaskStatus(string status)
    {
        taskStatusText.text = status;
    }
    
    // Method to update based on robot feedback
    public void OnTaskUpdate(TaskStatus status, string details)
    {
        UpdateTaskStatus($"Task '{currentTask}' - Status: {status}. {details}");
        
        if (status == TaskStatus.Completed || status == TaskStatus.Failed)
        {
            taskAssigned = false;
        }
    }
}

public enum TaskStatus
{
    Assigned,
    InProgress,
    Completed,
    Failed,
    Interrupted
}
```

## VR/AR Integration for Enhanced Interaction

### Virtual Reality Interfaces

VR can provide immersive robot control:

- Full body tracking for intuitive gesture control
- Spatial interfaces that match the robot's environment
- Immersive telepresence experiences

### Augmented Reality Interfaces

AR overlays can enhance real-world interaction:

- Information overlays on physical robots
- Gesture control in the robot's environment
- Mixed reality collaboration spaces

## Accessibility Considerations

### Universal Design

Design interfaces accessible to users with various abilities:

- Color-blind friendly palettes
- Keyboard navigation alternatives
- Screen reader compatibility
- Adjustable text sizes and contrast

### Adaptive Interfaces

Create interfaces that adapt to user preferences:

- Customizable control layouts
- Different interaction modes for different users
- Learning from user interaction patterns

## Performance Considerations

### Interface Responsiveness

Maintain smooth interaction:

- Optimize UI rendering
- Use object pooling for frequently created elements
- Implement efficient update strategies

### Network Latency Handling

For remote robot control:

- Predictive visualization
- Graceful degradation when connectivity is poor
- Clear indication of communication delays

## Best Practices for HRI in Unity

1. **User Testing**: Regularly test interfaces with actual users
2. **Consistency**: Maintain consistent interaction patterns across the application
3. **Error Prevention**: Design interfaces to prevent common errors
4. **Recovery**: Provide clear recovery paths when errors occur
5. **Scalability**: Design interfaces that work with multiple robots
6. **Documentation**: Provide clear instructions for complex interactions

## Summary

Human-robot interaction in Unity requires thoughtful design that balances intuitive control with safety and efficiency. By implementing well-designed interfaces, visualization systems, and feedback mechanisms, you can create digital twin experiences that make it natural and safe for humans to work with humanoid robots. The key is to understand both human and robot capabilities and to design interfaces that leverage the strengths of both while compensating for their limitations.