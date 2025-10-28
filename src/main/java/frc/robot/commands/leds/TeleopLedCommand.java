package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.DriverStation;
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
  private ClimbState climbState = ClimbState.STOWED;

  public TeleopLedCommand(RobotContainer container) {
    this.container = container;
    led = container.getLed();
    claw = container.getClaw();
    drive = container.getDrive();
    climber = container.getClimber();
    addRequirements(); // no requirements
  }

  @Override
  public void initialize() {
    led.commandOff().withName("Led Off Teleop Init").schedule();
  }

  @Override
  public void execute() {
    // this all runs in priority order

    // if can bus down, left yellow flash / right green flash
    if (!climber.isOK() && !driveCanbusDown) { // this should also check climbe roller and drive
      led.commandBlinkingState(LedState.kYellow, 0.25, LedStrip.LEFT)
          .asProxy()
          .alongWith(led.commandBlinkingState(LedState.kGreen, 0.25, LedStrip.RIGHT).asProxy())
          .withName("Led Drive CANBus off")
          .schedule();
      driveCanbusDown = true;
      return;
    } else if (!climber.isOK()) {
      return;
    } else if (climber.isOK()) {
      driveCanbusDown = false;
    }

    // climber flashing yellow coming out, solid yellow deployed, flashing green when climbing,
    // solid green when climbed
    if (Climber.getClimbState() == ClimbState.DEPLOYING && climbState != ClimbState.DEPLOYING) {
      led.commandBlinkingState(LedState.kYellow, 0.25).withName("Led Climb Deploying").schedule();
      climbState = ClimbState.DEPLOYING;
      return;
    } else if (Climber.getClimbState() == ClimbState.DEPLOYED
        && climbState != ClimbState.DEPLOYED) {
      led.commandSolidColor(LedState.kYellow).withName("Led Climb Deployed").schedule();
      climbState = ClimbState.DEPLOYED;
      return;
    } else if (Climber.getClimbState() == ClimbState.CLIMBING
        && climbState != ClimbState.CLIMBING) {
      led.commandBlinkingState(LedState.kGreen, 0.25).withName("Led Climb Climbing").schedule();
      climbState = ClimbState.CLIMBING;
      return;
    } else if (Climber.getClimbState() == ClimbState.CLIMBED && climbState != ClimbState.CLIMBED) {
      led.commandSolidColor(LedState.kGreen).withName("Led Climb Climbed").schedule();
      climbState = ClimbState.CLIMBED;
      return;
    }
    if (climbState != ClimbState.STOWED) {
      return;
    }

    // if can bus down, left yellow flash / right green flash
    if (!claw.isOK() && !miscCanbusDown) { // this should check a bunch of other subsystems too
      led.commandBlinkingState(LedState.kYellow, 0.25, LedStrip.RIGHT)
          .asProxy()
          .alongWith(led.commandBlinkingState(LedState.kGreen, 0.25, LedStrip.LEFT).asProxy())
          .withName("Led Misc CANBus off")
          .schedule();
      miscCanbusDown = true;
      return;
    } else if (!claw.isOK()) {
      return;
    } else if (claw.isOK()) {
      miscCanbusDown = false;
    }

    // has algae and has coral: solid white
    if (claw.hasAlgae() && CoralStateTracker.hasCoral()) {
      led.commandSolidColor(LedState.kWhite).withName("Led Both Game Pieces").schedule();
      return;
    }

    // has algae: teal
    if (claw.hasAlgae()) {
      led.commandSetTeal().withName("Led Has Algae").schedule();
      return;
    }
    // has coral: orange  (future: orange with different fill levels based on scoring height)
    if (CoralStateTracker.hasCoral()) {
      led.commandSetOrange().withName("Led Has Coral").schedule();
      return;
    }
    // else off
    led.commandOff().withName("Led Default Teleop").schedule();
  }

  @Override
  public boolean isFinished() {
    return !DriverStation.isTeleopEnabled();
  }

  @Override
  public void end(boolean interrupted) {
    led.commandOff().withName("Led Off Teleop End").schedule();
  }
}
