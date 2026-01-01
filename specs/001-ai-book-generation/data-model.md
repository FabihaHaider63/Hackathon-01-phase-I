# Data Model: Module 3: The AI-Robot Brain (NVIDIA Isaac™)

**Feature**: Module 3: The AI-Robot Brain (NVIDIA Isaac™)
**Created**: 2025-12-20
**Status**: Draft

## Entities

### IsaacSimEnvironment (Primary Entity)
- **Fields**:
  - id: string (unique identifier)
  - name: string (environment name)
  - description: string (brief description of the environment)
  - sceneGraph: object (representation of the 3D scene)
  - lightingConditions: object (lighting configuration)
  - physicsProperties: object (physics parameters)
  - sensorConfigurations: IsaacSimSensor[] (array of sensor configurations)
  - domainRandomization: object (domain randomization settings)
  - syntheticDataSettings: object (settings for synthetic data generation)
  - createdDate: Date (creation timestamp)
  - lastModified: Date (last modification timestamp)
  - compatibleRobots: string[] (list of compatible robot models)

- **Relationships**:
  - Parent Module (one-to-many with Module)
  - Child IsaacSimSensor (one-to-many)
  - Associated IsaacROSPipeline (one-to-one for sim-to-real)

- **Validation Rules**:
  - name: Required, max 100 characters
  - sceneGraph: Required, must be valid USD format
  - physicsProperties: Required, must pass validation against PhysX parameters

### IsaacROSPipeline
- **Fields**:
  - id: string (unique identifier)
  - name: string (pipeline name)
  - description: string (brief description of the pipeline)
  - perceptionNodes: IsaacROSNode[] (array of perception nodes)
  - vslamConfiguration: object (VSLAM algorithm settings)
  - sensorInterfaces: object (mapping of sensors to ROS interfaces)
  - performanceMetrics: object (timing and accuracy metrics)
  - createdDate: Date (creation timestamp)
  - lastModified: Date (last modification timestamp)
  - targetPlatform: string (target hardware platform)

- **Relationships**:
  - Parent Module (one-to-many with Module)
  - Child IsaacROSNode (one-to-many)
  - Associated IsaacSimEnvironment (one-to-one for sim-to-real)
  - Linked Nav2Configuration (one-to-one for navigation)

- **Validation Rules**:
  - name: Required
  - perceptionNodes: Required, minimum 1
  - performanceMetrics: Required, must meet real-time constraints

### IsaacROSNode
- **Fields**:
  - id: string (unique identifier)
  - name: string (node name)
  - type: string (node type: detection, tracking, slam, etc.)
  - inputs: string[] (input topic names)
  - outputs: string[] (output topic names)
  - configuration: object (node-specific parameters)
  - performance: object (timing and accuracy metrics)
  - createdDate: Date (creation timestamp)
  - lastModified: Date (last modification timestamp)
  - requiresGPU: boolean (whether node requires GPU acceleration)

- **Relationships**:
  - Parent IsaacROSPipeline (many-to-one)
  - CodeExamples (one-to-many with CodeExample)

- **Validation Rules**:
  - name: Required
  - inputs: Required, valid ROS 2 topic names
  - outputs: Required, valid ROS 2 topic names
  - performance: Required, must meet specified requirements

### Nav2Configuration
- **Fields**:
  - id: string (unique identifier)
  - name: string (configuration name)
  - description: string (brief description of the navigation setup)
  - planners: string[] (list of global planners configured)
  - controllers: string[] (list of local controllers configured)
  - recoveryBehaviors: string[] (list of recovery behaviors)
  - costmapConfiguration: object (costmap parameters)
  - humanoidSpecificParams: object (biped-specific navigation parameters)
  - createdDate: Date (creation timestamp)
  - lastModified: Date (last modification timestamp)

- **Relationships**:
  - Parent Module (one-to-many with Module)
  - Linked IsaacROSPipeline (one-to-one for sensor input)
  - Associated IsaacSimEnvironment (one-to-one for simulation)

- **Validation Rules**:
  - name: Required
  - planners: Required, minimum 1
  - controllers: Required, minimum 1
  - humanoidSpecificParams: Required for bipedal robots

### ReinforcementLearningModel
- **Fields**:
  - id: string (unique identifier)
  - name: string (model name)
  - algorithmType: string (PPO, DDPG, A2C, etc.)
  - sensorInputs: string[] (list of sensor inputs to the model)
  - actionSpace: object (definition of action space)
  - observationSpace: object (definition of observation space)
  - trainingEnvironment: IsaacSimEnvironment (simulator used for training)
  - preTrainedWeights: string (path to pre-trained model weights)
  - trainingMetrics: object (metrics during training)
  - createdDate: Date (creation timestamp)
  - lastModified: Date (last modification timestamp)

- **Relationships**:
  - Parent Module (one-to-many with Module)
  - Associated IsaacSimEnvironment (many-to-one for training)

- **Validation Rules**:
  - name: Required
  - algorithmType: Required, valid RL algorithm
  - actionSpace: Required, properly defined
  - observationSpace: Required, properly defined

### SyntheticDataset
- **Fields**:
  - id: string (unique identifier)
  - name: string (dataset name)
  - description: string (brief description of the dataset)
  - size: number (number of samples)
  - sensorModality: string (camera, LiDAR, IMU, etc.)
  - annotationType: string (bounding boxes, segmentation masks, etc.)
  - generatedFrom: IsaacSimEnvironment (source environment)
  - aiFramework: string (PyTorch, TensorFlow, etc.)
  - createdDate: Date (creation timestamp)
  - lastModified: Date (last modification timestamp)

- **Relationships**:
  - Parent Module (one-to-many with Module)
  - Source IsaacSimEnvironment (many-to-one)
  - UsedBy IsaacROSPipeline (many-to-many)

- **Validation Rules**:
  - name: Required
  - size: Positive number
  - sensorModality: Required, valid sensor type
  - annotationType: Required, valid annotation format

## State Transitions

### IsaacSimEnvironment Lifecycle
1. **Design**: Environment scene is created in Isaac Sim
2. **Configure**: Physics and lighting parameters are set
3. **Test**: Environment is validated for simulation use
4. **Deploy**: Environment is used for synthetic data generation or RL training

### State Transition Rules:
- Design → Configure: Manual trigger when scene is ready
- Configure → Test: Automatic when all parameters are set
- Test → Deploy: Manual trigger after successful validation

### IsaacROSPipeline Lifecycle
1. **Design**: Pipeline architecture is planned
2. **Implement**: Individual nodes are configured
3. **Validate**: Pipeline is tested in simulation
4. **Deploy**: Pipeline is deployed to real robot

### State Transition Rules:
- Design → Implement: Manual trigger when architecture is ready
- Implement → Validate: Manual trigger when all nodes are configured
- Validate → Deploy: Manual trigger after successful simulation tests

## Relationships

### IsaacSimEnvironment → IsaacROSPipeline (One-to-Many)
An environment can be used to train multiple perception pipelines.

### IsaacROSPipeline → IsaacROSNode (One-to-Many)
A pipeline contains multiple processing nodes.

### IsaacROSPipeline → Nav2Configuration (One-to-Many)
A perception pipeline can feed into multiple navigation configurations.

### IsaacSimEnvironment → SyntheticDataset (One-to-Many)
An environment can generate multiple synthetic datasets.

### IsaacSimEnvironment → ReinforcementLearningModel (One-to-Many)
An environment can be used to train multiple RL models.

### Nav2Configuration → ReinforcementLearningModel (Many-to-Many)
Navigation configurations may incorporate learned models.