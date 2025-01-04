# How to use Monologue

This fork of monologue essentially adds manual logging capabilities to our robot code.
There are 2 kinds of logging:

1. Logging with the @Logged annotation, done via epilogue(you learn this via the wpilib docs)
2. Manual log calls: like log(key, value).

Monologue accomplishes the latter in 2 ways:

### Logging under the same namespace as epilogue:
```java
class Arm extends SubsystemBase implements LogLocal {
	@Logged double x; // logged under robot/arm/x by epilogue
	
	@Override
    public void periodic() {
		log("y", 2.0); // logged under robot/arm/y by monologue
    }
}
```
Note 1: this can only be done for classes that can be found by epilogue.
Commands(and other objects returned by methods/functions) do not fit this criteria!

Note 2: LogLocal simply allows monologue's local log() functions to work. It is not required
for epilogue.

### Logging simple values:
```java
GlobalLog.log("a/b/c", 2.0); // logs under a/b/c
```

### Setting up monologue:
```java
class Robot extends TimedRobot implements LogLocal {
	public Robot() {
		Epilogue.bind(this);
		Monologue.setup(this, Epilogue.getConfig());
    }
}
```