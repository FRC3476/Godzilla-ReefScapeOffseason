package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LedConstants.LedStrip;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.climb.Climber;
import frc.robot.subsystems.climb.Climber.ClimbState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.end_effector.Claw;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.led.LedState;
import frc.robot.subsystems.superstructure.CoralStateTracker;
import org.littletonrobotics.junction.Logger;

public class TeleopLedCommand extends Command {
  private final RobotContainer container;
  private final Led led;
  private final Claw claw;
  private final DriveSubsystem drive;
  private final Climber climber;
  private final RobotState robotState;

  private enum TELEOP_LED_STATE {
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
    DEFAULT
  }

  private TELEOP_LED_STATE state = TELEOP_LED_STATE.NONE;
  private TELEOP_LED_STATE prevState = TELEOP_LED_STATE.NONE;

  // private boolean driveCanbusDown = false;
  // private boolean miscCanbusDown = false;
  // private ClimbState climbState = ClimbState.STOWED;

  public TeleopLedCommand(RobotContainer container, RobotState robotState) {
    this.container = container;
    this.robotState = robotState;
    led = container.getLed();
    claw = container.getClaw();
    drive = container.getDrive();
    climber = container.getClimber();
    addRequirements(led); // no requirements
  }

  @Override
  public void initialize() {
    // led.commandOff().withName("Led Off Teleop Init").schedule();
  }

  @Override
  public void execute() {
    // this all runs in priority order
    prevState = state;

    state = TELEOP_LED_STATE.DEFAULT;
    if (CoralStateTracker.hasCoral()) {
      switch (robotState.getStoredScorePosition().getCoralScoreLevel()) {
        case L1:
          state = TELEOP_LED_STATE.HAS_CORAL_L1;
          break;
        case L2:
          state = TELEOP_LED_STATE.HAS_CORAL_L2;
          break;
        case L3:
          state = TELEOP_LED_STATE.HAS_CORAL_L3;
          break;
        case L4:
          state = TELEOP_LED_STATE.HAS_CORAL_L4;
          break;
        default:
          state = TELEOP_LED_STATE.HAS_CORAL_L4;
          break;
      }
    }
    if (claw.hasAlgae()) {
      state = TELEOP_LED_STATE.HAS_ALGAE;
    }
    if (claw.hasAlgae() && CoralStateTracker.hasCoral()) {
      state = TELEOP_LED_STATE.HAS_CORAL_AND_ALGAE;
    }
    if (!claw.isOK()) {
      state = TELEOP_LED_STATE.CAN_DOWN_MISC;
    }
    if (Climber.getClimbState() == ClimbState.CLIMBED) {
      state = TELEOP_LED_STATE.CLIMB_CLIMBED;
    }
    if (Climber.getClimbState() == ClimbState.CLIMBING) {
      state = TELEOP_LED_STATE.CLIMB_CLIMBING;
    }
    if (Climber.getClimbState() == ClimbState.DEPLOYED) {
      state = TELEOP_LED_STATE.CLIMB_DEPLOYED;
    }
    if (Climber.getClimbState() == ClimbState.DEPLOYING) {
      state = TELEOP_LED_STATE.CLIMB_DEPLOYING;
    }
    if (!climber.isOK()) {
      state = TELEOP_LED_STATE.CAN_DOWN_DRIVE;
    }

    Logger.recordOutput("LED/Teleop/state", state);
    Logger.recordOutput("LED/Teleop/prevState", prevState);

    if (state != prevState) {
      switch (state) {
        case CAN_DOWN_DRIVE:
          led.commandBlinkingState(LedState.kYellow, 0.25, LedStrip.LEFT)
              .asProxy()
              .alongWith(led.commandBlinkingState(LedState.kGreen, 0.25, LedStrip.RIGHT).asProxy())
              .withName("Teleop LED: " + state.name())
              .schedule();
          break;
        case CLIMB_DEPLOYING:
          led.commandBlinkingState(LedState.kYellow, 0.25)
              .withName("Teleop LED: " + state.name())
              .schedule();
          break;
        case CLIMB_DEPLOYED:
          led.commandSolidColor(LedState.kYellow)
              .withName("Teleop LED: " + state.name())
              .schedule();
          break;
        case CLIMB_CLIMBING:
          led.commandBlinkingState(LedState.kGreen, 0.25)
              .withName("Teleop LED: " + state.name())
              .schedule();
          break;
        case CLIMB_CLIMBED:
          led.commandSolidColor(LedState.kGreen).withName("Teleop LED: " + state.name()).schedule();
          break;
        case CAN_DOWN_MISC:
          led.commandBlinkingState(LedState.kGreen, 0.25, LedStrip.LEFT)
              .asProxy()
              .alongWith(led.commandBlinkingState(LedState.kYellow, 0.25, LedStrip.RIGHT).asProxy())
              .withName("Teleop LED: " + state.name())
              .schedule();
          break;
        case HAS_CORAL_AND_ALGAE:
          led.commandSolidColor(LedState.kWhite).withName("Teleop LED: " + state.name()).schedule();
          break;
        case HAS_ALGAE:
          led.commandSetTeal().withName("Teleop LED: " + state.name()).schedule();
          break;
        case HAS_CORAL_L1:
          led.commandPercentageFull(() -> 7.0 / 16.0, LedState.kCOOrangeLed)
              .withName("Teleop LED: " + state.name())
              .schedule();
          break;
        case HAS_CORAL_L2:
          led.commandPercentageFull(() -> 10.0 / 16.0, LedState.kCOOrangeLed)
              .withName("Teleop LED: " + state.name())
              .schedule();
          break;
        case HAS_CORAL_L3:
          led.commandPercentageFull(() -> 13.0 / 16.0, LedState.kCOOrangeLed)
              .withName("Teleop LED: " + state.name())
              .schedule();
          break;
        case HAS_CORAL_L4:
          led.commandPercentageFull(() -> 16.0 / 16.0, LedState.kCOOrangeLed)
              .withName("Teleop LED: " + state.name())
              .schedule();
          break;
        default:
          led.commandOff().withName("Teleop Led Default").schedule();
          break;
      }
    }

    // if can bus down, left yellow flash / right green flash
    // if (!climber.isOK() && !driveCanbusDown) { // this should also check climbe roller and drive
    //   led.commandBlinkingState(LedState.kYellow, 0.25, LedStrip.LEFT)
    //       .asProxy()
    //       .alongWith(led.commandBlinkingState(LedState.kGreen, 0.25, LedStrip.RIGHT).asProxy())
    //       .withName("Led Drive CANBus off")
    //       .schedule();
    //   driveCanbusDown = true;
    //   return;
    // } else if (!climber.isOK()) {
    //   return;
    // } else {
    //   driveCanbusDown = false;
    // }

    // // climber flashing yellow coming out, solid yellow deployed, flashing green when climbing,
    // // solid green when climbed
    // if (Climber.getClimbState() == ClimbState.DEPLOYING && climbState != ClimbState.DEPLOYING) {
    //   led.commandBlinkingState(LedState.kYellow, 0.25).withName("Led Climb
    // Deploying").schedule();
    //   climbState = ClimbState.DEPLOYING;
    //   return;
    // } else if (Climber.getClimbState() == ClimbState.DEPLOYED
    //     && climbState != ClimbState.DEPLOYED) {
    //   led.commandSolidColor(LedState.kYellow).withName("Led Climb Deployed").schedule();
    //   climbState = ClimbState.DEPLOYED;
    //   return;
    // } else if (Climber.getClimbState() == ClimbState.CLIMBING
    //     && climbState != ClimbState.CLIMBING) {
    //   led.commandBlinkingState(LedState.kGreen, 0.25).withName("Led Climb Climbing").schedule();
    //   climbState = ClimbState.CLIMBING;
    //   return;
    // } else if (Climber.getClimbState() == ClimbState.CLIMBED && climbState != ClimbState.CLIMBED)
    // {
    //   led.commandSolidColor(LedState.kGreen).withName("Led Climb Climbed").schedule();
    //   climbState = ClimbState.CLIMBED;
    //   return;
    // }
    // if (climbState != ClimbState.STOWED) {
    //   return;
    // }

    // // if can bus down, left yellow flash / right green flash
    // if (!claw.isOK() && !miscCanbusDown) { // this should check a bunch of other subsystems too
    //   led.commandBlinkingState(LedState.kYellow, 0.25, LedStrip.RIGHT)
    //       .asProxy()
    //       .alongWith(led.commandBlinkingState(LedState.kGreen, 0.25, LedStrip.LEFT).asProxy())
    //       .withName("Led Misc CANBus off")
    //       .schedule();
    //   miscCanbusDown = true;
    //   return;
    // } else if (!claw.isOK()) {
    //   return;
    // } else {
    //   miscCanbusDown = false;
    // }

    // // has algae and has coral: solid white
    // if (claw.hasAlgae() && CoralStateTracker.hasCoral()) {
    //   led.commandSolidColor(LedState.kWhite).withName("Led Both Game Pieces").schedule();
    //   return;
    // }

    // // has algae: teal
    // if (claw.hasAlgae()) {
    //   led.commandSetTeal().withName("Led Has Algae").schedule();
    //   return;
    // }
    // // has coral: orange
    // if (CoralStateTracker.hasCoral()) {
    //   // led.commandSetOrange().withName("Led Has Coral").schedule();
    //   if (robotState.getStoredScorePosition().getCoralScoreLevel() == ScoreLevel.L1) {
    //     led.commandPercentageFull(() -> 7.0 / 16.0, LedState.kCOOrangeLed)
    //         .withName("Led Has Coral L1")
    //         .schedule();
    //   }
    //   if (robotState.getStoredScorePosition().getCoralScoreLevel() == ScoreLevel.L2) {
    //     led.commandPercentageFull(() -> 10.0 / 16.0, LedState.kCOOrangeLed)
    //         .withName("Led Has Coral L2")
    //         .schedule();
    //   }
    //   if (robotState.getStoredScorePosition().getCoralScoreLevel() == ScoreLevel.L3) {
    //     led.commandPercentageFull(() -> 13.0 / 16.0, LedState.kCOOrangeLed)
    //         .withName("Led Has Coral L3")
    //         .schedule();
    //   }
    //   if (robotState.getStoredScorePosition().getCoralScoreLevel() == ScoreLevel.L4) {
    //     led.commandPercentageFull(() -> 16.0 / 16.0, LedState.kCOOrangeLed)
    //         .withName("Led Has Coral L4")
    //         .schedule();
    //   }
    //   return;
    // }
    // // else off
    // led.commandOff().withName("Led Default Teleop").schedule();
  }

  @Override
  public boolean isFinished() {
    return !DriverStation.isTeleopEnabled();
  }

  @Override
  public void end(boolean interrupted) {
    // led.commandOff().withName("Led Off Teleop End").schedule();
  }
}
