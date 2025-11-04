# AutoPilot Drive to Pose Command - Testing Guide

## Overview

This guide will help you test the newly created `DriveToPoseAutopilotCommand` that uses the 3414 AutoPilot library for smooth and accurate drive-to-pose functionality.

## What Was Added

### 1. AutoPilot Vendordep (`vendordeps/Autopilot.json`)
- Added the AutoPilot library dependency (version 2025.1.0)
- Maven repository: https://maven.therekrab.org/releases/

### 2. Constants (`Constants.java`)
Added AutoPilot-specific constants in `DriveConstants`:
- `AUTOPILOT_MAX_ACCELERATION`: 5.0 m/s² - Maximum acceleration for motion profiling
- `AUTOPILOT_MAX_JERK`: 2.0 m/s³ - Maximum jerk for smooth motion
- `AUTOPILOT_ERROR_XY_METERS`: 2 inches - Position tolerance
- `AUTOPILOT_ERROR_THETA_DEGREES`: 0.5 degrees - Rotation tolerance
- `AUTOPILOT_BEELINE_RADIUS_METERS`: 8 inches - Radius for switching to direct path
- `AUTOPILOT_HEADING_KP/KI/KD`: PID constants for heading control

### 3. Command Class (`commands/DriveToPoseAutopilotCommand.java`)
A new command that:
- Uses the AutoPilot library for motion profiling and path following
- Accepts either a `Pose2d` target or a `Supplier<Pose2d>` for dynamic targets
- Provides comprehensive telemetry logging via AdvantageKit
- Includes an `atTarget()` trigger for command chaining
- Automatically stops the robot when the command ends

### 4. Test Buttons in ElasticTabs (`humanControls/ElasticTabs.java`)
Added two test buttons in the "Drive" tab:
- **"AutoPilot: Drive to Reef Pole"**: Drives to the closest reef pole with proper offset
- **"AutoPilot: Drive Forward 2m"**: Simple test that drives 2 meters forward from current position

### 5. Named Commands (`auto/NamedCommandsSetup.java`)
Added two named commands for use in PathPlanner autonomous routines:
- **"AutoPilotLeftPoleAlign"**: Drive to left reef pole with AutoPilot
- **"AutoPilotRightPoleAlign"**: Drive to right reef pole with AutoPilot

## Testing Instructions

### Prerequisites
1. Ensure the robot code has been built successfully (already done)
2. Deploy the code to your robot or run in simulation
3. Have AdvantageScope or another dashboard ready to monitor telemetry

### Test 1: Simple Forward Movement Test
1. **Open the Dashboard** (AdvantageScope or similar)
2. **Navigate to the "Drive" tab** in Elastic Dashboard
3. **Place the robot in an open area** with at least 3 meters of clear space ahead
4. **Click and hold "AutoPilot: Drive Forward 2m"**
5. **Observe**:
   - Robot should smoothly accelerate forward
   - Motion should be smooth with controlled jerk
   - Robot should decelerate and stop approximately 2 meters from starting position
   - Rotation should remain constant

**Expected Results**:
- ✅ Smooth acceleration and deceleration
- ✅ Robot travels approximately 2 meters
- ✅ Maintains original heading
- ✅ Stops within tolerance (2 inches)

### Test 2: Reef Pole Alignment Test
1. **Ensure the robot pose is accurately estimated** (use vision or manual reset)
2. **Navigate to the "Drive" tab** in Elastic Dashboard
3. **Set the target scoring position** (left, right, or center pole)
4. **Click and hold "AutoPilot: Drive to Reef Pole"**
5. **Observe**:
   - Robot should drive toward the target reef pole
   - Should maintain appropriate offset (perpendicular to pole)
   - Should smoothly adjust both translation and rotation
   - Should stop when within tolerance

**Expected Results**:
- ✅ Robot reaches the reef pole with correct offset
- ✅ Robot orientation matches target pose
- ✅ Smooth motion profile throughout
- ✅ Command completes successfully (returns `isFinished()`)

### Test 3: AdvantageKit Telemetry Verification
Monitor these values in AdvantageScope:
- `Commands/DriveToPoseAutopilotCommand/CurrentPose`: Current robot pose
- `Commands/DriveToPoseAutopilotCommand/TargetPose`: Target pose
- `Commands/DriveToPoseAutopilotCommand/VelocityX`: X velocity command
- `Commands/DriveToPoseAutopilotCommand/VelocityY`: Y velocity command
- `Commands/DriveToPoseAutopilotCommand/TargetAngle`: Target heading angle
- `Commands/DriveToPoseAutopilotCommand/AtTarget`: Boolean indicating if at target
- `Commands/DriveToPoseAutopilotCommand/Active`: Command active status

**Expected Results**:
- ✅ Velocities should ramp smoothly (not jump suddenly)
- ✅ `AtTarget` should become true when within tolerance
- ✅ All telemetry values should be reasonable and not NaN/infinite

### Test 4: Autonomous Integration Test
1. **Open PathPlanner** and create or edit an auto routine
2. **Add a new command** using one of the AutoPilot named commands:
   - `AutoPilotLeftPoleAlign`
   - `AutoPilotRightPoleAlign`
3. **Run the autonomous routine**
4. **Observe** the robot executing the AutoPilot command

**Expected Results**:
- ✅ Command integrates seamlessly with PathPlanner
- ✅ 3 second timeout works correctly
- ✅ Robot completes alignment or times out appropriately

## Tuning Guide

If the robot behavior is not ideal, you can adjust these constants in `Constants.java`:

### Motion Profile Tuning
- **Increase `AUTOPILOT_MAX_ACCELERATION`** if the robot is too slow to accelerate
- **Decrease `AUTOPILOT_MAX_ACCELERATION`** if the robot jerks or skids
- **Increase `AUTOPILOT_MAX_JERK`** for more aggressive motion (less smooth)
- **Decrease `AUTOPILOT_MAX_JERK`** for smoother motion (slower response)

### Position Tolerance Tuning
- **Increase `AUTOPILOT_ERROR_XY_METERS`** if the robot oscillates around the target
- **Decrease `AUTOPILOT_ERROR_XY_METERS`** for tighter positioning (may take longer)
- **Increase `AUTOPILOT_ERROR_THETA_DEGREES`** if rotation oscillates
- **Decrease `AUTOPILOT_ERROR_THETA_DEGREES`** for more precise heading

### Path Following Tuning
- **Increase `AUTOPILOT_BEELINE_RADIUS_METERS`** to switch to direct path sooner
- **Decrease `AUTOPILOT_BEELINE_RADIUS_METERS`** for more conservative approach

### Heading Control Tuning
- **Increase `AUTOPILOT_HEADING_KP`** if heading response is too slow
- **Decrease `AUTOPILOT_HEADING_KP`** if heading oscillates
- **Add `AUTOPILOT_HEADING_KD`** (currently 0.0) for damping if needed

## Comparison with Existing Commands

| Feature | DriveToPosePIDCommand | GarageDriveToPoseCommand | DriveToPoseAutopilotCommand |
|---------|----------------------|--------------------------|----------------------------|
| Motion Profiling | Simple PID | Profiled PID | AutoPilot Library |
| Jerk Control | ❌ No | ❌ No | ✅ Yes |
| Smooth Acceleration | ⚠️ Limited | ✅ Yes | ✅ Yes (Better) |
| LED Integration | ❌ No | ✅ Yes | ❌ No (can be added) |
| Telemetry | ⚠️ Basic | ✅ Extensive | ✅ Extensive |
| Best Use Case | Simple movements | Competition alignment | Smooth autonomous paths |

## Troubleshooting

### Robot doesn't move
- **Check**: Is the drive subsystem properly initialized?
- **Check**: Are there any subsystem conflicts?
- **Check**: Is the robot enabled?
- **Verify**: Check console logs for AutoPilot library errors

### Robot moves erratically
- **Lower** `AUTOPILOT_MAX_ACCELERATION` and `AUTOPILOT_MAX_JERK`
- **Check** that pose estimation is accurate
- **Verify** chassis speeds are being reported correctly

### Robot doesn't stop at target
- **Check** that `isFinished()` is being called and returns true
- **Increase** error tolerances slightly
- **Verify** the target pose is correct in telemetry

### Command never finishes
- **Check** error tolerances - they may be too tight
- **Verify** the robot can physically reach the target pose
- **Add** a timeout to the command (already included in named commands)

## Next Steps

1. **Test thoroughly** in simulation before deploying to hardware
2. **Tune constants** based on your robot's characteristics
3. **Integrate** into competition autonomous routines
4. **Compare performance** with existing drive commands
5. **Document** any robot-specific tuning you discover

## Additional Resources

- [AutoPilot Documentation](https://therekrab.github.io/autopilot/)
- [AutoPilot Examples](https://therekrab.github.io/autopilot/examples.html)
- [Chief Delphi Discussion](https://www.chiefdelphi.com/t/autopilot-with-pathplanner/470081)

## Support

If you encounter issues with the AutoPilot library itself:
- Check the [AutoPilot GitHub Issues](https://github.com/TheRekrabCircus/AutoPilot)
- Ask on the [Chief Delphi forums](https://www.chiefdelphi.com/)
- Review the AutoPilot documentation for API changes

---

**Created**: November 4, 2025  
**Library Version**: AutoPilot 2025.1.0  
**FRC Season**: 2025

