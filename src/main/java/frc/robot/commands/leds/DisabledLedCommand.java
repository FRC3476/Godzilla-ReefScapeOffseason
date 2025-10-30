package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LedConstants;
import frc.robot.Field.FieldUtils;
import frc.robot.RobotContainer;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.led.LedState;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class DisabledLedCommand extends Command {
  private final RobotContainer container;
  private final Led led;

  private enum DISABLED_LED_STATE {
    NONE,
    LOW_BATTERY,
    RED,
    BLUE,
    DEFAULT
  }

  private DISABLED_LED_STATE state = DISABLED_LED_STATE.NONE;
  private DISABLED_LED_STATE prevState = DISABLED_LED_STATE.NONE;

  private static final LoggedTunableNumber lowBatteryThreshold =
      new LoggedTunableNumber("LED/Low Battery Threshold", LedConstants.kLowBatteryThresholdVolts);

  public DisabledLedCommand(RobotContainer container) {
    this.container = container;
    led = container.getLed();
    addRequirements(); // no requirements
  }

  @Override
  public void initialize() {
    led.commandOff().withName("Led Off Disabled Init").schedule();
  }

  @Override
  public void execute() {
    prevState = state;

    // lowest check is highest priority
    state = DISABLED_LED_STATE.DEFAULT;
    if (FieldUtils.getAlliance() == Alliance.Red && DriverStation.isFMSAttached()) {
      state = DISABLED_LED_STATE.RED;
    }
    if (FieldUtils.getAlliance() == Alliance.Blue && DriverStation.isFMSAttached()) {
      state = DISABLED_LED_STATE.BLUE;
    }
    if (RobotController.getBatteryVoltage() < lowBatteryThreshold.get()) {
      state = DISABLED_LED_STATE.LOW_BATTERY;
    }

    Logger.recordOutput("LED/Disabled/state", state);
    Logger.recordOutput("LED/Disabled/prevState", prevState);

    if (state != prevState) {
      switch (state) {
        case LOW_BATTERY:
          led.commandBlinkingState(LedState.kLowBattery, 0.125)
              .withName("Disabled LED: " + state.name())
              .schedule();
          break;
        case RED:
          led.commandLarson(LedState.kRed).withName("Disabled LED: " + state.name()).schedule();
          break;
        case BLUE:
          led.commandLarson(LedState.kBlue).withName("Disabled LED: " + state.name()).schedule();
          break;
        case DEFAULT:
          led.commandColorflowCO().withName("Disabled LED: " + state.name()).schedule();
          break;
        default:
          led.commandOff().withName("Disabled LED default").schedule();
          break;
      }
    }

    // if (RobotController.getBatteryVoltage() < lowBatteryThreshold.get()
    //     && !(state == DISABLED_LED_STATE.LOW_BATTERY)) {
    //   led.commandBlinkingState(LedState.kLowBattery, 0.125)
    //       .withName(Disabled LED Low Battery")
    //       .schedule();
    //   state = DISABLED_LED_STATE.LOW_BATTERY;
    //   return;
    // } else if (RobotController.getBatteryVoltage() < lowBatteryThreshold.get()) {
    //   return;
    // } else if (state == DISABLED_LED_STATE.LOW_BATTERY) {
    //   state = DISABLED_LED_STATE.NONE;
    // }

    // try {
    //   // this all runs in priority order
    //   if (FieldUtils.getAlliance() == Alliance.Blue
    //       && DriverStation.isFMSAttached()
    //       && !(state == DISABLED_LED_STATE.BLUE)) {
    //     led.commandLarson(LedState.kBlue).withName("Disabled LED Blue").schedule();
    //     state = DISABLED_LED_STATE.BLUE;
    //     return;
    //   } else if (FieldUtils.getAlliance() == Alliance.Red
    //       && DriverStation.isFMSAttached()
    //       && !(state == DISABLED_LED_STATE.RED)) {
    //     led.commandLarson(LedState.kRed).withName("Disabled LED Red").schedule();
    //     state = DISABLED_LED_STATE.RED;
    //     return;
    //   }
    // } catch (Exception e) {
    //   System.out.println("Exception details: " + e.toString());
    // }

    // // else orange and teal
    // if (!(state == DISABLED_LED_STATE.DEFAULT)) {
    //   led.commandColorflowCO().schedule();
    //   state = DISABLED_LED_STATE.DEFAULT;
    // }
  }

  @Override
  public boolean isFinished() {
    return !DriverStation.isDisabled();
  }

  @Override
  public void end(boolean interrupted) {
    // led.commandOff().withName("Led Off Disabled End").schedule();
  }
}
