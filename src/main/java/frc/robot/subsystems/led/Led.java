package frc.robot.subsystems.led;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.RobotState;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.end_effector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.CoralStateTracker;
import frc.robot.subsystems.superstructure.CoralStateTracker.CoralPosition;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Led extends SubsystemBase {
  private final LedIO io;

  public record PercentageSetpoint(double pct, LedState color) {}

  public Led(final LedIO io, RobotState state) {
    this.io = io;
  }

  @Override
  public void periodic() {
    super.periodic();

    RobotState.setLedState(getCurrentState());
    Logger.recordOutput(
        "LED/currentCommand",
        (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());
  }

  public LedState getCurrentState() {
    return io.getCurrentState();
  }

  /* change runOnce to run in case we have to keep setting the LED color periodically? */
  public Command commandSolidColor(LedState state) {
    return run(() -> setSolidColor(state)).ignoringDisable(true).withName("LED Solid Color");
  }

  public Command commandSolidColor(Supplier<LedState> state) {
    return run(() -> setSolidColor(state.get())).ignoringDisable(true).withName("LED Solid Color");
  }

  public Command commandSolidPattern(LedState[] states) {
    return run(() -> setSolidPattern(states)).ignoringDisable(true).withName("LED Solid Pattern");
  }

  public Command commandPercentageFull(DoubleSupplier percentageFull, LedState state) {
    return run(() -> setPercentageFull(percentageFull.getAsDouble(), state)).ignoringDisable(true);
  }

  public Command commandPercentageFull(Supplier<PercentageSetpoint> percentageSupplier) {
    return run(() ->
            setPercentageFull(percentageSupplier.get().pct, percentageSupplier.get().color))
        .ignoringDisable(true);
  }

  public Command commandBlinkingState(
      LedState stateOne, LedState stateTwo, double durationOne, double durationTwo) {
    return new SequentialCommandGroup(
            Commands.runOnce(() -> setSolidColor(stateOne)),
            new WaitCommand(durationOne),
            Commands.runOnce(() -> setSolidColor(stateTwo)),
            new WaitCommand(durationTwo))
        .repeatedly()
        .ignoringDisable(true)
        .withName("Blinking LED command");
  }

  public Command commandBlinkingStateWithoutScheduler(
      LedState stateOne, LedState stateTwo, double durationOne, double durationTwo) {
    var state =
        new Object() {
          public boolean color1 = true;
          public double timestamp = Timer.getFPGATimestamp();
        };
    return Commands.runOnce(
            () -> {
              state.color1 = true;
              state.timestamp = Timer.getFPGATimestamp();
            })
        .andThen(
            commandSolidColor(
                () -> {
                  if (state.color1 && state.timestamp + durationOne <= Timer.getFPGATimestamp()) {
                    state.color1 = false;
                    state.timestamp = Timer.getFPGATimestamp();
                  } else if (!state.color1
                      && state.timestamp + durationTwo <= Timer.getFPGATimestamp()) {
                    state.color1 = true;
                    state.timestamp = Timer.getFPGATimestamp();
                  }

                  if (state.color1) {
                    return stateOne;
                  } else {
                    return stateTwo;
                  }
                }))
        .ignoringDisable(true)
        .withName("Blinking LED command");
  }

  public Command commandBlinkingState(LedState stateOne, LedState stateTwo, double duration) {
    return commandBlinkingState(stateOne, stateTwo, duration, duration).ignoringDisable(true);
  }

  private void setSolidColor(LedState state) {
    io.writePixels(state);
  }

  private void setSolidPattern(LedState[] states) {
    io.writePixels(states);
  }

  private void setPercentageFull(double percentageFull, LedState state) {
    LedState[] pixels = new LedState[Constants.LEDConstants.kMaxLEDCount / 2];
    for (int i = 0; i < pixels.length; i++) {
      if (i < pixels.length * MathUtil.clamp(percentageFull, 0.0, 1.0)) {
        pixels[i] = state;
      }
    }
  }

  @SuppressWarnings("unused")
  private LedState[] mirror(LedState[] pixels) {
    LedState[] fullPixels = new LedState[Constants.LEDConstants.kMaxLEDCount];

    for (int i = Constants.LEDConstants.kCandleLEDCount;
        i
            < Constants.LEDConstants.kCandleLEDCount
                + (Constants.LEDConstants.kNonCandleLEDCount / 2);
        i++) {
      fullPixels[fullPixels.length - i - 2] = pixels[i];
      fullPixels[i] = pixels[i];
    }

    return fullPixels;
  }

  public Command createDefaultCommand(Intake intake, EndEffector endEffector, Elevator elevator) {
    return commandSolidColor(() -> {

      double batteryVoltage = RobotController.getBatteryVoltage();
      if (batteryVoltage < Constants.LEDConstants.kLowBatteryThresholdVolts) {
        return LedState.kLowBattery;
      }

      if (DriverStation.isDisabled()) {
        if (DriverStation.getAlliance().isPresent()) {
          Alliance alliance = DriverStation.getAlliance().get();
          return alliance == Alliance.Red ? LedState.kRed : LedState.kBlue;
        }
        return LedState.kWhite; 
      }

      CoralPosition coralPosition = CoralStateTracker.getCurrentPosition();
      switch (coralPosition) {
        case AT_INTAKE:
          return LedState.kGreen; 
        case GOING_TO_FEEDER:
          return LedState.kYellow; 
        case AT_FEEDER:
          return LedState.kOrange; 
        case AT_FIRST_END_EFFECTOR:
          return LedState.kCyan; 
        case AT_SECOND_END_EFFECTOR:
          return LedState.kBlue; 
        case STAGED_IN_END_EFFECTOR:
          return LedState.kPurple; 
        case NONE:
        default:
          break; 
      }

      IntakeState intakeState = intake.getCurrentState();
      switch (intakeState) {
        case SCORING:
        case SCORING_PREP:
          return LedState.kCoralMode; 
        case REJECT_CORAL:
          return LedState.kRed; 
        case HAND_OFF:
          return LedState.kYellow; 
        case JAM_DETECTED:
          return LedState.kPink; 
        default:
          break; 
      }

      if (elevator.elevatorObjectTrigger.getAsBoolean()) {
        return LedState.kPink; 
      }

      if (endEffector.isCoralInEndeffector()) {
        return LedState.kCoralMode; 
      }
      if (endEffector.hasAlgae()) {
        return LedState.kAlgaeMode; 
      }

      if (DriverStation.isAutonomous()) {
        return LedState.kPurple; 
      }

      if (DriverStation.isTeleop()) {
        double time = Timer.getFPGATimestamp();
        double brightness = (Math.sin(time * 2.0) + 1.0) / 2.0; // https://www.desmos.com/calculator/qt2phfeona
        int scaledBrightness = (int) (brightness * 255);
        return new LedState(scaledBrightness, scaledBrightness, scaledBrightness);
      }

      return LedState.kCOOrange;
    }).withName("LED Default State Control");
  }

  public void setupSpecialPatterns(Intake intake, Elevator elevator) {
    // These were just added so that some of the functions are locally used
    new Trigger(() -> intake.getCurrentState() == IntakeState.SCORING_PREP)
        .whileTrue(commandBlinkingState(LedState.kWhite, LedState.kCoralMode, 0.3));
        
    new Trigger(() -> elevator.elevatorObjectTrigger.getAsBoolean())
        .whileTrue(commandBlinkingState(LedState.kRed, LedState.kOff, 0.1));
        
    new Trigger(() -> CoralStateTracker.getCurrentPosition() == CoralPosition.STAGED_IN_END_EFFECTOR)
        .whileTrue(commandSolidPattern(getStagingLedPattern(elevator)));

    new Trigger(() -> DriverStation.isTeleop() && DriverStation.getMatchTime() <= 30.0 && DriverStation.getMatchTime() > 0.0)
        .whileTrue(commandSolidColor(() -> getAllianceEndgameColor()));
  }

  public LedState[] getStagingLedPattern(Elevator elevator) {
    double elevatorPosition = elevator.getCurrentPosition();
    
    if (elevatorPosition >= Constants.ElevatorConstants.ELEVATOR_L4_SETPOINT_INCH) { 
      return createFullLedArray(LedState.kPurple);
    } else if (elevatorPosition >= Constants.ElevatorConstants.ELEVATOR_L3_SETPOINT_INCH) { 
      return LedState.kL3StagingLeds;
    } else if (elevatorPosition >= Constants.ElevatorConstants.ELEVATOR_L2_SETPOINT_INCH) { 
      return LedState.kL2StagingLeds;
    } else {
      return LedState.kL2StagingLeds; 
    }
  }

  private LedState[] createFullLedArray(LedState color) {
    LedState[] fullArray = new LedState[Constants.LEDConstants.kMaxLEDCount];
    for (int i = 0; i < fullArray.length; i++) {
      fullArray[i] = color;
    }
    return fullArray;
  }

  public static LedState getAllianceEndgameColor() {
    if (DriverStation.getAlliance().isPresent()) {
      Alliance alliance = DriverStation.getAlliance().get();

      return alliance == Alliance.Red ? LedState.kRed : LedState.kBlue;
    }
    return LedState.kYellow; 
  }
}
