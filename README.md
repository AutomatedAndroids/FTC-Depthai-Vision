# FTC Depthai Vision

This package is designed to replace the existing FTC:Vision library built into your robot, as a relatively drop-in replacement. Additionally, it features a collection of systems for working depthai, and hopefully more camera systems in the future.

### WARNING: THIS PACKAGE IS _NOT_ MEANT FOR PRODUCTION SYSTEMS.
## Installation
[![](https://jitpack.io/v/AutomatedAndroids/FTC-Depthai-Vision.svg)](https://jitpack.io/#AutomatedAndroids/FTC-Depthai-Vision)

This package is published through Jitpack.

### Installation using Gradle:
Note: Not using gradle? See the jitpack linked above.

To use it, add the following to the repository section of your `build.gradle`, part of the larger FTC project; 
```
maven { url 'https://jitpack.io' }
```

Then add the following to the dependencies in your `build.gradle`. Make sure that this is placed in the  `build.gradle` file in the larger `FTC` project, rather than in just your `:Teamcode` module.
```
dependencies {
  implementation 'com.github.AutomatedAndroids:FTC-Depthai-Vision:Tag'
}
```
## Notes
And That's it!

Just so you know, some refactoring will be required if you wish to use this project. Specifically, you'll have to remove the original `FTC:VISION` implementation in your `build.dependencies.gradle` file, and you'll need to adjust any files that are already using the vision system accordingly.
