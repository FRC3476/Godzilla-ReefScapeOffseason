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

public class DisabledLedCommand extends Command {
  private final RobotContainer container;
  private final Led led;

  private enum RUNNING {
    NONE,
    LOW_BATTERY,
    RED,
    BLUE,
    DEFAULT
  }

  private RUNNING running = RUNNING.NONE;

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
    if (RobotController.getBatteryVoltage() < lowBatteryThreshold.get()
        && !(running == RUNNING.LOW_BATTERY)) {
      led.commandBlinkingState(LedState.kLowBattery, 0.125)
          .withName("Disabled LED Low Battery")
          .schedule();
      running = RUNNING.LOW_BATTERY;
      return;
    } else if (RobotController.getBatteryVoltage() < lowBatteryThreshold.get()) {
      return;
    } else if (running == RUNNING.LOW_BATTERY) {
      running = RUNNING.NONE;
    }

    try {
      // this all runs in priority order
      if (FieldUtils.getAlliance() == Alliance.Blue
          && DriverStation.isFMSAttached()
          && !(running == RUNNING.BLUE)) {
        led.commandLarson(LedState.kBlue).withName("Disabled LED Blue").schedule();
        running = RUNNING.BLUE;
        return;
      } else if (FieldUtils.getAlliance() == Alliance.Red
          && DriverStation.isFMSAttached()
          && !(running == RUNNING.RED)) {
        led.commandLarson(LedState.kRed).withName("Disabled LED Red").schedule();
        running = RUNNING.RED;
        return;
      }
    } catch (Exception e) {
      System.out.println("Exception details: " + e.toString());
    }

    // else orange and teal
    if (!(running == RUNNING.DEFAULT)) {
      led.commandColorflowCO().schedule();
      running = RUNNING.DEFAULT;
    }
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
