# 78 AIR STRIKE's 2025 Robot

## Getting Started

IntelliJ is the recommended IDE to use for this repository due to the use of Kotlin.

Install IntelliJ (Community Edition is free) and open the repository.

## Formatting

Formatting is done through the `ktfmt` plugin in IntelliJ, or the spotless plugin in gradle.

To run formatting in the command line, use the following command:

Windows:

```bash
$ gradlew spotlessApply
```

macOS/Linux:

```bash
$ ./gradlew spotlessApply
```

## Building and Deploying

You can build and deploy the robot code using IntelliJ, or the command line.

To build the robot code, use the following command:

```bash
$ gradlew build
$ gradlew deploy
```

macOS/Linux:

```bash
$ ./gradlew build
$ ./gradlew deploy
```

## Creating Commands

There is a [kotlin delegate](https://kotlinlang.org/docs/delegated-properties.html) that ensure the creation of commands
is done in a way that creates a new command instance with each use.
This is necessary to prevent reusing instances of commands in multiple places.

Using the delegate:

```kotlin
val exampleCommand by command { PrintCommand("Hello, World!") }
```

## Creating Subsystems

Subsystems should be created as `object`s to ensure that there is only one instance of each subsystem.

```kotlin
object Arm : SubsystemBase()
```

Accessing the subsystem can be done using the object name

```kotlin
Arm.raise()
```

