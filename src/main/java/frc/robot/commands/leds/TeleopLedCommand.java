package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LedConstants.LedStrip;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climb.Climber;
import frc.robot.subsystems.climb.Climber.ClimbState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.end_effector.Claw;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.led.LedState;
import frc.robot.subsystems.superstructure.CoralStateTracker;

public class TeleopLedCommand extends Command {
  private final RobotContainer container;
  private final Led led;
  private final Claw claw;
  private final DriveSubsystem drive;
  private final Climber climber;

  private boolean driveCanbusDown = false;
  private boolean miscCanbusDown = false;

  public TeleopLedCommand(RobotContainer container) {
    this.container = container;
    led = container.getLed();
    claw = container.getClaw();
    drive = container.getDrive();
    climber = container.getClimber();
  }

  @Override
  public void execute() {
    // this all runs in priority order

    // if can bus down, left yellow flash / right green flash
    if (!climber.isOK() && !driveCanbusDown) { // this should also check climbe roller and drive
      led.commandBlinkingState(LedState.kYellow, 0.25, LedStrip.LEFT)
          .asProxy()
          .alongWith(led.commandBlinkingState(LedState.kGreen, 0.25, LedStrip.RIGHT).asProxy())
          .schedule();
      driveCanbusDown = true;
      return;
    } else if (!climber.isOK()) {
      return;
    } else if (climber.isOK()) {
      driveCanbusDown = false;
    }

    // climber flashing yellow coming out, solid yellow deployed, flashing green when climbing, solid green when climbed
    if (climber.getClimbState() == ClimbState.DEPLOYING){
      led.commandBlinkingState(LedState.kYellow, 0.25).schedule();
      return;
    } else if (climber.getClimbState() == ClimbState.DEPLOYED){
      led.commandSolidColor(LedState.kYellow).schedule();
      return;
    } else if (climber.getClimbState() == ClimbState.CLIMBING){
      led.commandBlinkingState(LedState.kGreen, 0.25).schedule();
      return;
    } else if (climber.getClimbState() == ClimbState.CLIMBED){
      led.commandSolidColor(LedState.kGreen).schedule();
      return;
    }

    // if can bus down, left yellow flash / right green flash
    if (!claw.isOK() && !miscCanbusDown) { // this should check a bunch of other subsystems too
      led.commandBlinkingState(LedState.kYellow, 0.25, LedStrip.RIGHT)
          .asProxy()
          .alongWith(led.commandBlinkingState(LedState.kGreen, 0.25, LedStrip.LEFT).asProxy())
          .schedule();
      miscCanbusDown = true;
      return;
    } else if (!claw.isOK()) {
      return;
    } else if (claw.isOK()) {
      miscCanbusDown = false;
    }

    // has algae and has coral: solid white
    if (claw.hasAlgae() && CoralStateTracker.hasCoral()){
      led.commandSolidColor(LedState.kWhite).schedule();
      return;
    }

    // has algae: teal
    if (claw.hasAlgae()){
      led.commandSolidColor(LedState.kCOTealLed).schedule();
      return;
    }
    // has coral: orange  (future: orange with different fill levels based on scoring height)
    if (CoralStateTracker.hasCoral()){
      led.commandSolidColor(LedState.kCOOrangeLed);
    }
    // else off
    led.commandOff().schedule();
  }

  @Override
  public boolean isFinished() {
    return !DriverStation.isTeleopEnabled();
  }

  @Override
  public void end(boolean interrupted) {}
}
