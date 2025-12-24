# State Machine Diagrams

This document contains MermaidJS state diagrams for each subsystem's state machine.

## 1. SuperStructure State Machine

Controls the arm assembly (pivot, elevator, wrist) positioning for scoring game pieces.

```mermaid
stateDiagram-v2
    [*] --> Home
    Home --> CoralStation: !hasGamePiece
    Home --> FloorAlgae: !hasGamePiece
    Home --> HighAlgae: !hasGamePiece
    Home --> LowAlgae: !hasGamePiece
    Home --> CoralScoring: hasCoral
    Home --> Processor: hasAlgae
    Home --> Net: hasAlgae
    Home --> ReadyToClimb
    CoralStation --> Home
    FloorAlgae --> Home
    HighAlgae --> Home
    HighAlgae --> LowAlgae
    LowAlgae --> Home
    LowAlgae --> HighAlgae
    Processor --> Home
    Net --> Home
    ReadyToClimb --> Home
    ReadyToClimb --> FullyClimbed

    state CoralScoring {
        L1
        L2
        L3
        L4
        note right of L1: All L states can transition\nto any other L state
    }
    CoralScoring --> Home
    FullyClimbed --> [*]
```

## 2. Intake State Machine

Manages game piece acquisition and control using torque/current sensing.

```mermaid
stateDiagram-v2
    [*] --> Idle
    Idle --> IntakeCoral
    Idle --> IntakeAlgae
    IntakeCoral --> Idle
    IntakeCoral --> HoldCoral: holdingCoral detected
    HoldCoral --> EjectCoral: SuperStructure.atPosition
    EjectCoral --> Idle
    EjectCoral --> IntakeCoral
    IntakeAlgae --> Idle
    IntakeAlgae --> HoldAlgae: holdingAlgae detected
    HoldAlgae --> ProcessAlgae
    NetAlgae --> Idle
    ProcessAlgae --> Idle
```

## 3. Climber State Machine

Controls the end-game climbing mechanism.

```mermaid
stateDiagram-v2
    [*] --> Retracted
    Retracted --> Extended: SuperStructure.state == FullyClimbed
    Extended --> [*]
    note right of Extended: (6 inches)
```

## 4. Chassis (Drivetrain) State Machine

Manages swerve drivetrain control with auto-alignment features.

```mermaid
stateDiagram-v2
    [*] --> FieldCentric
    FieldCentric --> RobotCentric
    FieldCentric --> AutoAlignCoral: holdingCoral && L2-L4\n[debounced 0.3s]
    FieldCentric --> AutoAlignAlgae: !holdingAlgae && algae button\n[debounced 0.3s]
    RobotCentric --> FieldCentric
    RobotCentric --> AutoAlignCoral: holdingCoral && L2-L4\n[debounced 0.3s]
    RobotCentric --> AutoAlignAlgae: !holdingAlgae && algae button\n[debounced 0.3s]
    AutoAlignCoral --> FieldCentric
    AutoAlignAlgae --> FieldCentric
    note right of AutoAlignCoral: Drives to closest branch\nwith coral offset
    note right of AutoAlignAlgae: Drives to closest reef
```

## 5. LED Subsystem State Machine

Provides visual feedback through addressable LEDs.

```mermaid
stateDiagram-v2
    [*] --> Disabled
    Disabled: Color = White
    Idle: Color = Black (off)
    GamePieceAcquired: Color = Green
    Aligned: Color = Blue
    Idle --> GamePieceAcquired: Intake holding game piece
    GamePieceAcquired --> Idle: Intake not holding game piece
    note right of Aligned: Set externally by Chassis\nwhen auto-aligned
```

