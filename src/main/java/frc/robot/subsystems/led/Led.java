package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LedConstants.LedStrip;
import frc.robot.Field.FieldUtils;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.climb.Climber;
import frc.robot.subsystems.climb.Climber.ClimbState;
import frc.robot.subsystems.superstructure.CoralStateTracker;
import frc.robot.util.COColor;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Led extends SubsystemBase {
  private final LedIO io;
  private final RobotState state;
  private static final LoggedTunableNumber LedsOn = new LoggedTunableNumber("LED/Num On", 0);

  public enum DEFAULT_LED_STATE {
    NONE,
    CAN_DOWN_DRIVE,
    CAN_DOWN_MISC,
    CLIMB_DEPLOYING,
    CLIMB_DEPLOYED,
    CLIMB_CLIMBING,
    CLIMB_CLIMBED,
    HAS_CORAL_AND_ALGAE,
    HAS_ALGAE,
    HAS_CORAL_L1,
    HAS_CORAL_L2,
    HAS_CORAL_L3,
    HAS_CORAL_L4,
    LOW_BATTERY,
    DISABLED_RED,
    DISABLED_BLUE,
    DEFAULT_TELEOP,
    DEFAULT_DISABLED,
    GARAGE_DRIVE_ALIGNED;

    public boolean isDefault() {
      switch (this) {
        case NONE,
            CAN_DOWN_DRIVE,
            CAN_DOWN_MISC,
            CLIMB_DEPLOYING,
            CLIMB_DEPLOYED,
            CLIMB_CLIMBING,
            CLIMB_CLIMBED,
            HAS_CORAL_AND_ALGAE,
            HAS_ALGAE,
            HAS_CORAL_L1,
            HAS_CORAL_L2,
            HAS_CORAL_L3,
            HAS_CORAL_L4,
            LOW_BATTERY,
            DISABLED_RED,
            DISABLED_BLUE,
            DEFAULT_TELEOP,
            DEFAULT_DISABLED:
          return true;
        default:
          return false;
      }
    }
  }

  private DEFAULT_LED_STATE defaultState = DEFAULT_LED_STATE.NONE;
  private DEFAULT_LED_STATE prevDefaultState = DEFAULT_LED_STATE.NONE;
  private static final LoggedTunableNumber lowBatteryThreshold =
      new LoggedTunableNumber(
          "LED/Low Battery Threshold", Constants.LedConstants.kLowBatteryThresholdVolts);

  public record PercentageSetpoint(double pct, COColor color) {}

  public Led(final LedIO io, RobotState state) {
    this.io = io;
    this.state = state;
  }

  @Override
  public void periodic() {
    super.periodic();

    RobotState.setLedState(getCurrentState());
    Logger.recordOutput(
        "LED/currentCommand",
        (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());
    Logger.recordOutput("LED/state", defaultState);
    Logger.recordOutput("LED/prevState", prevDefaultState);
  }

  public COColor getCurrentState() {
    return io.getCurrentState();
  }

  public int getLedsOn() {
    return (int) Math.round(LedsOn.get());
  }

  /* change runOnce to run in case we have to keep setting the LED color periodically? */
  public Command commandSolidColor(COColor state) {
    return runOnce(() -> setSolidColor(state, LedStrip.BOTH))
        .ignoringDisable(true)
        .withName("LED Solid Color");
  }

  public Command commandSolidColor(COColor state, LedStrip strip) {
    return runOnce(() -> setSolidColor(state, strip))
        .ignoringDisable(true)
        .withName("LED Solid Color");
  }

  public Command commandSolidColor(Supplier<COColor> state) {
    return runOnce(() -> setSolidColor(state.get()))
        .ignoringDisable(true)
        .withName("LED Solid Color");
  }

  public Command commandSolidColorNumLeds(COColor state, Supplier<Integer> numLeds) {
    return runOnce(() -> setSolidColorNumLeds(state, numLeds))
        .ignoringDisable(true)
        .withName("LED Solid Color Num Leds");
  }

  public Command commandOff() {
    return commandSolidColor(COColor.kOff).withName("Led Off");
  }

  public Command commandSetTeal() {
    return commandSolidColor(COColor.kCOTealLed).withName("Led Teal");
  }

  public Command commandSetOrange() {
    return commandSolidColor(COColor.kCOOrangeLed).withName("Led Orange");
  }

  public Command commandSolidPattern(COColor[] states) {
    return runOnce(() -> setSolidPattern(states))
        .ignoringDisable(true)
        .withName("LED Solid Pattern");
  }

  public Command commandPercentageFull(DoubleSupplier percentageFull, COColor state) {
    return runOnce(() -> setPercentageFull(percentageFull.getAsDouble(), state))
        .ignoringDisable(true);
  }

  public Command commandPercentageFull(Supplier<PercentageSetpoint> percentageSupplier) {
    return run(() ->
            setPercentageFull(percentageSupplier.get().pct, percentageSupplier.get().color))
        .ignoringDisable(true);
  }

  public Command commandBlinkingState(COColor state, double duration) {
    return this.runOnce(() -> blinkingState(state, duration, LedStrip.BOTH)).ignoringDisable(true);
  }

  public Command commandBlinkingState(COColor state, double duration, LedStrip strip) {
    return this.runOnce(() -> blinkingState(state, duration, strip)).ignoringDisable(true);
  }

  public void blinkingState(COColor state, double duration) {
    blinkingState(state, duration, LedStrip.BOTH);
  }

  public void blinkingState(COColor state, double duration, LedStrip strip) {
    this.io.blink(state, duration, strip);
  }

  public Command commandFire() {
    return this.runOnce(() -> this.io.fire())
        .ignoringDisable(true)
        .ignoringDisable(true)
        .withName("LED Fire");
  }

  public Command commandRainbow() {
    return this.runOnce(() -> this.io.rainbow())
        .ignoringDisable(true)
        .ignoringDisable(true)
        .withName("LED Rainbow");
  }

  public Command commandColorflowCO() {
    return this.runOnce(() -> this.io.colorflowCO()).ignoringDisable(true).withName("LED LarsonCO");
  }

  public Command commandLarson(COColor state) {
    return this.runOnce(() -> this.io.larson(state)).ignoringDisable(true).withName("LED Larson");
  }

  public Command commandTwinkle(COColor state, boolean off) {
    return this.runOnce(() -> this.io.twinkle(state, off))
        .ignoringDisable(true)
        .withName("LED Twinkle");
  }

  private void setSolidColor(COColor state) {
    io.writePixels(state, LedStrip.BOTH);
  }

  private void setSolidColor(COColor state, LedStrip strip) {
    io.writePixels(state, strip);
  }

  private void setSolidColorNumLeds(COColor state, Supplier<Integer> numLeds) {
    System.out.println(numLeds);
    io.writeNumPixels(state, numLeds);
  }

  private void setSolidPattern(COColor[] states) {
    io.writePixels(states);
  }

  public void setLedState(DEFAULT_LED_STATE state) {
    defaultState = state;
    Logger.recordOutput("LED/Force Set State", state);
  }

  public Command ledDefault(RobotContainer container, RobotState robotState) {
    return this.run(
            () -> {
              Logger.recordOutput("LED/StartingState", defaultState);
              Logger.recordOutput("LED/isDefault", defaultState.isDefault());
              if (defaultState.isDefault()) prevDefaultState = defaultState;

              if (!defaultState.isDefault()) {
                if (prevDefaultState != defaultState) {
                  switch (defaultState) {
                    case GARAGE_DRIVE_ALIGNED:
                      io.rainbow();
                      break;
                    default:
                      setSolidColor(COColor.kOff);
                      break;
                  }
                  prevDefaultState = defaultState;
                }
              } else if (DriverStation.isTeleopEnabled()) {
                defaultState = DEFAULT_LED_STATE.DEFAULT_TELEOP;
                if (CoralStateTracker.hasCoral()) {
                  switch (robotState.getStoredScorePosition().getCoralScoreLevel()) {
                    case L1:
                      defaultState = DEFAULT_LED_STATE.HAS_CORAL_L1;
                      break;
                    case L2:
                      defaultState = DEFAULT_LED_STATE.HAS_CORAL_L2;
                      break;
                    case L3:
                      defaultState = DEFAULT_LED_STATE.HAS_CORAL_L3;
                      break;
                    case L4:
                      defaultState = DEFAULT_LED_STATE.HAS_CORAL_L4;
                      break;
                    default:
                      defaultState = DEFAULT_LED_STATE.HAS_CORAL_L4;
                      break;
                  }
                }
                if (container.getClaw().hasAlgae()) {
                  defaultState = DEFAULT_LED_STATE.HAS_ALGAE;
                }
                if (container.getClaw().hasAlgae() && CoralStateTracker.hasCoral()) {
                  defaultState = DEFAULT_LED_STATE.HAS_CORAL_AND_ALGAE;
                }
                if (!container.getClaw().isOK()) {
                  defaultState = DEFAULT_LED_STATE.CAN_DOWN_MISC;
                }
                if (Climber.getClimbState() == ClimbState.CLIMBED) {
                  defaultState = DEFAULT_LED_STATE.CLIMB_CLIMBED;
                }
                if (Climber.getClimbState() == ClimbState.CLIMBING) {
                  defaultState = DEFAULT_LED_STATE.CLIMB_CLIMBING;
                }
                if (Climber.getClimbState() == ClimbState.DEPLOYED) {
                  defaultState = DEFAULT_LED_STATE.CLIMB_DEPLOYED;
                }
                if (Climber.getClimbState() == ClimbState.DEPLOYING) {
                  defaultState = DEFAULT_LED_STATE.CLIMB_DEPLOYING;
                }
                if (!container.getClimber().isOK()) {
                  defaultState = DEFAULT_LED_STATE.CAN_DOWN_DRIVE;
                }

                Logger.recordOutput("LED/TeleopState", defaultState);
                Logger.recordOutput("LED/TeleopPrevState", prevDefaultState);

                if (defaultState != prevDefaultState) {
                  switch (defaultState) {
                    case CAN_DOWN_DRIVE:
                      blinkingState(COColor.kYellow, 0.25, LedStrip.LEFT);
                      blinkingState(COColor.kGreen, 0.25, LedStrip.RIGHT);
                      break;
                    case CLIMB_DEPLOYING:
                      blinkingState(COColor.kYellow, 0.1);
                      break;
                    case CLIMB_DEPLOYED:
                      setSolidColor(COColor.kYellow);
                      break;
                    case CLIMB_CLIMBING:
                      blinkingState(COColor.kGreen, 0.1);
                      break;
                    case CLIMB_CLIMBED:
                      setSolidColor(COColor.kGreen);
                      break;
                    case CAN_DOWN_MISC:
                      blinkingState(COColor.kGreen, 0.25, LedStrip.LEFT);
                      blinkingState(COColor.kYellow, 0.25, LedStrip.RIGHT);
                      break;
                    case HAS_CORAL_AND_ALGAE:
                      setSolidColor(COColor.kWhite);
                      break;
                    case HAS_ALGAE:
                      setSolidColor(COColor.kCOTealLed);
                      break;
                    case HAS_CORAL_L1:
                      setPercentageFull(7.0 / 16.0, COColor.kCOOrangeLed);
                      break;
                    case HAS_CORAL_L2:
                      setPercentageFull(10.0 / 16.0, COColor.kCOOrangeLed);
                      break;
                    case HAS_CORAL_L3:
                      setPercentageFull(13.0 / 16.0, COColor.kCOOrangeLed);
                      break;
                    case HAS_CORAL_L4:
                      setPercentageFull(16.0 / 16.0, COColor.kCOOrangeLed);
                      break;
                    default:
                      setSolidColor(COColor.kOff);
                      break;
                  }
                }
              } else if (DriverStation.isAutonomousEnabled()) {
                io.fire();
              } else { // Disabled
                // low battery debounce
                if (!(defaultState == DEFAULT_LED_STATE.LOW_BATTERY
                    && RobotController.getBatteryVoltage() < lowBatteryThreshold.get() + 0.1)) {
                  // lowest check is highest priority
                  defaultState = DEFAULT_LED_STATE.DEFAULT_DISABLED;
                  if (FieldUtils.getAlliance() == Alliance.Red && DriverStation.isFMSAttached()) {
                    defaultState = DEFAULT_LED_STATE.DISABLED_RED;
                  }
                  if (FieldUtils.getAlliance() == Alliance.Blue && DriverStation.isFMSAttached()) {
                    defaultState = DEFAULT_LED_STATE.DISABLED_BLUE;
                  }
                  if (RobotController.getBatteryVoltage() < lowBatteryThreshold.get()
                      && !DriverStation.isFMSAttached()) {
                    defaultState = DEFAULT_LED_STATE.LOW_BATTERY;
                  }
                }

                Logger.recordOutput("LED/DisabledState", defaultState);
                Logger.recordOutput("LED/DisabledPrevState", prevDefaultState);

                if (defaultState != prevDefaultState) {
                  switch (defaultState) {
                    case LOW_BATTERY:
                      blinkingState(COColor.kLowBattery, 0.125);
                      break;
                    case DISABLED_RED:
                      this.io.larson(COColor.kRed);
                      break;
                    case DISABLED_BLUE:
                      this.io.larson(COColor.kBlue);
                      break;
                    case DEFAULT_DISABLED:
                      this.io.colorflowCO();
                      break;
                    default:
                      setSolidColor(COColor.kOff);
                      break;
                  }
                }
              }
            })
        .ignoringDisable(true);
  }

  // private void setPercentageFull(double percentageFull, LedState state) {
  //   LedState[] pixels = new LedState[Constants.LedConstants.kMaxLEDCount / 2];
  //   for (int i = 0; i < pixels.length; i++) {
  //     if (i < pixels.length * MathUtil.clamp(percentageFull, 0.0, 1.0)) {
  //       pixels[i] = state;
  //     }
  //   }
  // }

  private void setPercentageFull(double percent, COColor state) {
    this.io.percentageFull(percent, state);
  }

  @SuppressWarnings("unused")
  private COColor[] mirror(COColor[] pixels) {
    COColor[] fullPixels = new COColor[Constants.LedConstants.kMaxLEDCount];

    for (int i = Constants.LedConstants.kCandleLEDCount;
        i
            < Constants.LedConstants.kCandleLEDCount
                + (Constants.LedConstants.kNonCandleLEDCount / 2);
        i++) {
      fullPixels[fullPixels.length - i - 2] = pixels[i];
      fullPixels[i] = pixels[i];
    }

    return fullPixels;
  }
}
