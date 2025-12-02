package frc.robot.humanControls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EndEffectorConstants.ClawState;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.LedConstants.LedStrip;
import frc.robot.Field.FieldUtils;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToCoralCommand;
import frc.robot.commands.DriveToPosePIDCommand;
import frc.robot.subsystems.climb.ClimbRoller;
import frc.robot.subsystems.climb.Climber;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.end_effector.ClawOld;
import frc.robot.subsystems.end_effector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.led.LedState;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.Controls.ElasticButton.ElasticTab;
import frc.robot.util.PoseUtils;
import java.util.HashMap;
import java.util.Map;

public class ElasticTabs {
  private final RobotContainer container;
  private final Elevator elevator;
  private final DriveSubsystem drive;
  private final Superstructure superstructure;
  private final Intake intake;
  private final Climber climber;
  private final ClimbRoller climbRoller;
  private final RobotState robotState;
  private final EndEffector endEffector;
  private final ClawOld claw;
  private final Led led;
  private final Vision vision;
  private final Map<String, ElasticTab> elasticTabMap = new HashMap<>();

  public ElasticTabs(RobotContainer container, RobotState robotState) {
    this.container = container;
    this.robotState = robotState;
    drive = container.getDrive();
    superstructure = container.getSuperStructure();
    elevator = container.getElevator();
    intake = container.getIntake();
    climber = container.getClimber();
    climbRoller = container.getClimbRoller();
    endEffector = container.getEndEffector();
    claw = container.getClaw();
    led = container.getLed();
    vision = container.getVision();
    buildElasticTabs();
  }

  private void buildElasticTabs() {
    buildIntakeTab();
    buildEndEffectorTab();
    buildElevatorTab();
    buildSuperstructureTab();
    buildClimberTab();
    buildDriveTab();
    buildLedTab();
    buildTestTab();
  }

  private void buildIntakeTab() {
    String key = "Intake";
    ElasticTab tab = new ElasticTab(key);
    elasticTabMap.put(key, tab);

    // Configure the while-held behavior
    tab.addButton("Roller Forward (While Held)")
        .setupWhileHeldCommand(intake.intakeFWD(), intake.intakeSTOP());
    tab.addButton("Roller Reverse (While Held)")
        .setupWhileHeldCommand(intake.intakeRVS(), intake.intakeSTOP());
    tab.addButton("Pivot Up (While Held)")
        .setupWhileHeldCommand(intake.pivotManualTestForward(), intake.pivotStop());
    tab.addButton("Pivot Down (While Held)")
        .setupWhileHeldCommand(intake.pivotManualTestReverse(), intake.pivotStop());
    tab.addButton("Feeder In (While Held)")
        .setupWhileHeldCommand(intake.feederFWD(), intake.feederSTOP());
    tab.addButton("Feeder Out (While Held)")
        .setupWhileHeldCommand(intake.feederRVS(), intake.feederSTOP());

    tab.addButton("Pivot Up (When Pressed)").setupOnPressCommand(intake.setPivotUp());
    tab.addButton("Pivot Intake Position (When Pressed)")
        .setupOnPressCommand(intake.setPivotDown());
    tab.addButton("Pivot Scoring Position (When Pressed)")
        .setupOnPressCommand(intake.setPivotScoring());
    tab.addButton("Pivot Zero Position (When Pressed)")
        .setupOnPressCommandIgnoringDisabled(intake.zeroPivotAtPivotUp());

    // Configure intake state button triggers
    tab.addButton("STOW (When Pressed)")
        .setupOnPressCommand(intake.setIntakeStateCommand(IntakeState.STOW));
    tab.addButton("Intake (When Pressed)")
        .setupOnPressCommand(intake.setIntakeStateCommand(IntakeState.INTAKE));
    tab.addButton("Reject Coral (When Pressed)")
        .setupOnPressCommand(intake.setIntakeStateCommand(IntakeState.REJECT_CORAL));
    tab.addButton("Idle (When Pressed)")
        .setupOnPressCommand(intake.setIntakeStateCommand(IntakeState.IDLE));
    tab.addButton("Hand Off (When Pressed)")
        .setupOnPressCommand(intake.setIntakeStateCommand(IntakeState.HAND_OFF));
    tab.addButton("Scoring (When Pressed)")
        .setupOnPressCommand(intake.setIntakeStateCommand(IntakeState.SCORING));
    tab.addButton("Scoring Prep (When Pressed)")
        .setupOnPressCommand(intake.setIntakeStateCommand(IntakeState.SCORING_PREP));
    tab.addButton("Manual Control (When Pressed)")
        .setupOnPressCommand(intake.setIntakeStateCommand(IntakeState.NONE));
  }

  private void buildEndEffectorTab() {
    // Get the NetworkTable for the EndEffector tab
    String key = "EndEffector";
    ElasticTab tab = new ElasticTab(key);
    elasticTabMap.put(key, tab);

    // Configure the while-held behavior
    tab.addButton("Roller Forward (While Held)")
        .setupWhileHeldCommand(claw.rollerFWD(), claw.rollerSTOP());
    tab.addButton("Roller Reverse (While Held)")
        .setupWhileHeldCommand(claw.rollerRVS(), claw.rollerSTOP());
    tab.addButton("Roller Hold (While Held)")
        .setupWhileHeldCommand(claw.holdAlgae(), claw.holdAlgae());
    tab.addButton("Pivot Up (While Held)")
        .setupWhileHeldCommand(endEffector.pivotUP(), endEffector.pivotSTOP());
    tab.addButton("Pivot Down (While Held)")
        .setupWhileHeldCommand(endEffector.pivotDOWN(), endEffector.pivotSTOP());

    tab.addButton("Claw Score").setupOnPressCommand(claw.setClawStateCommand(ClawState.SCORING));
    tab.addButton("Claw Score L1")
        .setupOnPressCommand(claw.setClawStateCommand(ClawState.SCORING_L1));
    tab.addButton("Claw Algae Hold").setupOnPressCommand(claw.setClawStateCommand(ClawState.ALGAE));
    tab.addButton("Claw None").setupOnPressCommand(claw.setClawStateCommand(ClawState.NONE));
    tab.addButton("Pivot Safe Up (When Pressed)")
        .setupOnPressCommand(
            endEffector.motionMagicSetpointCommand(
                () -> Constants.EndEffectorConstants.MAX_ANGLE_ROTATIONS));
    tab.addButton("Pivot Fully Up (When Pressed)")
        .setupOnPressCommand(
            endEffector.motionMagicSetpointCommand(
                () -> Constants.EndEffectorConstants.MAX_SAFE_ANGLE_ROTATIONS));
    tab.addButton("Pivot Safe Down (When Pressed)")
        .setupOnPressCommand(
            endEffector.motionMagicSetpointCommand(
                () -> Constants.EndEffectorConstants.MIN_SAFE_ANGLE_ROTATIONS));
    tab.addButton("Pivot Fully Down (When Pressed)")
        .setupOnPressCommand(
            endEffector.motionMagicSetpointCommand(
                () -> Constants.EndEffectorConstants.MIN_ANGLE_ROTATIONS));
    tab.addButton("Pivot Middle (When Pressed)")
        .setupOnPressCommand(
            endEffector.motionMagicSetpointCommand(
                () ->
                    (Constants.EndEffectorConstants.MIN_SAFE_ANGLE_ROTATIONS
                            + Constants.EndEffectorConstants.MAX_SAFE_ANGLE_ROTATIONS)
                        / 2));
    tab.addButton("Pivot Zero (When Pressed)")
        .setupOnPressCommandIgnoringDisabled(endEffector.setPivotZero());
  }

  private void buildElevatorTab() {
    String key = "Elevator";
    ElasticTab tab = new ElasticTab(key);
    elasticTabMap.put(key, tab);

    // Configure the while-held behavior
    tab.addButton("Elevator Up (While Held)")
        .setupWhileHeldCommand(elevator.elevatorUP(), elevator.elevatorSTOP());
    tab.addButton("Elevator Down (While Held)")
        .setupWhileHeldCommand(elevator.elevatorDWN(), elevator.elevatorSTOP());

    tab.addButton("Elevator L2 (When Pressed)")
        .setupOnPressCommand(
            elevator.positionSetpointCommand(() -> SuperstructureState.L2_AIM.getElevatorHeight()));
    tab.addButton("Elevator L3 (When Pressed)")
        .setupOnPressCommand(
            elevator.positionSetpointCommand(() -> SuperstructureState.L3_AIM.getElevatorHeight()));
    tab.addButton("Elevator L4 (When Pressed)")
        .setupOnPressCommand(
            elevator.positionSetpointCommand(() -> SuperstructureState.L4_AIM.getElevatorHeight()));
    tab.addButton("Elevator Down Pos (When Pressed)")
        .setupOnPressCommand(
            elevator.positionSetpointCommand(() -> SuperstructureState.STOW.getElevatorHeight()));
    tab.addButton("Zero the Elevator (When Pressed)")
        .setupOnPressCommandIgnoringDisabled(
            Commands.runOnce(() -> elevator.setCurrentPosition(0)));
  }

  private void buildSuperstructureTab() {
    String key = "Superstructure";
    ElasticTab tab = new ElasticTab(key);
    elasticTabMap.put(key, tab);

    // Wire triggers to superstructure state commands
    tab.addButton("STOW")
        .setupOnPressCommand(
            superstructure.setStateCommand(SuperstructureState.STOW, "Elastic Set STOW"));
    tab.addButton("STOW_CORAL")
        .setupOnPressCommand(
            superstructure.setStateCommand(
                SuperstructureState.STOW_CORAL, "Elastic Set STOW_CORAL"));
    tab.addButton("STOW_ALGAE")
        .setupOnPressCommand(
            superstructure.setStateCommand(
                SuperstructureState.STOW_ALGAE, "Elastic Set STOW_ALGAE"));
    tab.addButton("INTAKE_CORAL")
        .setupOnPressCommand(
            superstructure.setStateCommand(
                SuperstructureState.INTAKE_CORAL, "Elastic Set INTAKE_CORAL"));
    tab.addButton("INTAKE_CORAL_L1")
        .setupOnPressCommand(
            superstructure.setStateCommand(
                SuperstructureState.INTAKE_CORAL_L1, "Elastic Set INTAKE_CORAL_L1"));
    tab.addButton("FEED")
        .setupOnPressCommand(
            superstructure.setStateCommand(SuperstructureState.FEED, "Elastic Set FEED"));
    tab.addButton("L1_FADEAWAY")
        .setupOnPressCommand(
            superstructure.setStateCommand(
                SuperstructureState.L1_FADEAWAY, "Elastic Set L1_FADEAWAY"));
    tab.addButton("L2_FADEAWAY")
        .setupOnPressCommand(
            superstructure.setStateCommand(
                SuperstructureState.L2_FADEAWAY, "Elastic Set L2_FADEAWAY"));
    tab.addButton("L3_FADEAWAY")
        .setupOnPressCommand(
            superstructure.setStateCommand(
                SuperstructureState.L3_FADEAWAY, "Elastic Set L3_FADEAWAY"));
    tab.addButton("L4_FADEAWAY")
        .setupOnPressCommand(
            superstructure.setStateCommand(
                SuperstructureState.L4_FADEAWAY, "Elastic Set L4_FADEAWAY"));
    tab.addButton("L1_AIM")
        .setupOnPressCommand(
            superstructure.setStateCommand(SuperstructureState.L1_AIM, "Elastic Set L1_AIM"));
    tab.addButton("L2_AIM")
        .setupOnPressCommand(
            superstructure.setStateCommand(SuperstructureState.L2_AIM, "Elastic Set L2_AIM"));
    tab.addButton("L3_AIM")
        .setupOnPressCommand(
            superstructure.setStateCommand(SuperstructureState.L3_AIM, "Elastic Set L3_AIM"));
    tab.addButton("L4_AIM")
        .setupOnPressCommand(
            superstructure.setStateCommand(SuperstructureState.L4_AIM, "Elastic Set L4_AIM"));
    tab.addButton("L2_AWAY_FROM_REEF")
        .setupOnPressCommand(
            superstructure.setStateCommand(
                SuperstructureState.L2_AWAY_FROM_REEF, "Elastic Set L2_AWAY_FROM_REEF"));
    tab.addButton("L3_AWAY_FROM_REEF")
        .setupOnPressCommand(
            superstructure.setStateCommand(
                SuperstructureState.L3_AWAY_FROM_REEF, "Elastic Set L3_AWAY_FROM_REEF"));
    tab.addButton("L4_AWAY_FROM_REEF")
        .setupOnPressCommand(
            superstructure.setStateCommand(
                SuperstructureState.L4_AWAY_FROM_REEF, "Elastic Set L4_AWAY_FROM_REEF"));
    tab.addButton("ALGAE_HIGH_INTAKE")
        .setupOnPressCommand(
            superstructure.setStateCommand(
                SuperstructureState.ALGAE_HIGH_INTAKE, "Elastic Set ALGAE_HIGH_INTAKE"));
    tab.addButton("ALGAE_LOW_INTAKE")
        .setupOnPressCommand(
            superstructure.setStateCommand(
                SuperstructureState.ALGAE_LOW_INTAKE, "Elastic Set ALGAE_LOW_INTAKE"));
    tab.addButton("PROCESSOR_AIM")
        .setupOnPressCommand(
            superstructure.setStateCommand(
                SuperstructureState.PROCESSOR_AIM, "Elastic Set PROCESSOR_AIM"));
    tab.addButton("BARGE_AIM_CENTER")
        .setupOnPressCommand(
            superstructure.setStateCommand(
                SuperstructureState.BARGE_AIM_CENTER, "Elastic Set BARGE_AIM_CENTER"));
    tab.addButton("BARGE_AIM_BACKWARD")
        .setupOnPressCommand(
            superstructure.setStateCommand(
                SuperstructureState.BARGE_AIM_BACKWARD, "Elastic Set BARGE_AIM_BACKWARD"));
    tab.addButton("BARGE_AIM_FORWARD")
        .setupOnPressCommand(
            superstructure.setStateCommand(
                SuperstructureState.BARGE_AIM_FORWARD, "Elastic Set BARGE_AIM_FORWARD"));
  }

  private void buildDriveTab() {
    String key = "Drive";
    ElasticTab tab = new ElasticTab(key);
    elasticTabMap.put(key, tab);

    // tab.addButton("Characterize
    // Feedforward").setupWhileHeldCommand(DriveCommands.feedforwardCharacterization(drive));
    // tab.addButton("Characterize Slip Current").setupWhileHeldCommand(
    //     Commands.print("running slip current test")
    //     .andThen(DriveCommands.slipCurrentCharacterization(drive)));
    tab.addButton("Characterize Wheel Radius")
        .setupWhileHeldCommand(DriveCommands.wheelRadiusCharacterization(drive));
    // tab.addButton("Drive Stop X").setupOnPressCommand(
    //     Commands.runOnce(drive::stopWithX, drive));
    // tab.addButton("Drive Forward").setupWhileHeldCommand(
    //     Commands.run(() -> drive.runVelocity(new ChassisSpeeds(1, 0.0, 0.0))));
    // tab.addButton("Drive Turn Clockwise").setupWhileHeldCommand(
    //     Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 1))));

    tab.addButton("Drive to Coral Pose")
        .setupWhileHeldCommand(new DriveToCoralCommand(drive, vision));
    /*new PathfindToPoseCommand(
    drive,
    () ->
        PoseUtils.getPerpendicularOffsetPose(
            FieldConstants.redReefCD.rightPole.getPose(), 0.65)));*/

    // FieldUtils.getClosestReefPole().getPose(), 0.65)));

    tab.addButton("Auto Align to Closest Pole")
        .setupWhileHeldCommand(
            new DriveToPosePIDCommand(
                drive,
                () -> {
                  Pose2d targetPose;
                  switch (robotState.getStoredScorePosition().getCoralBranch()) {
                    case LEFT:
                      targetPose = FieldUtils.getClosestReef().leftPole.getPose();
                    case RIGHT:
                      targetPose = FieldUtils.getClosestReef().rightPole.getPose();
                    default:
                      targetPose = FieldUtils.getClosestReefPole().getPose();
                  }
                  return PoseUtils.getPerpendicularOffsetPose(
                      targetPose, DriveConstants.AUTO_ALIGN_PERPENDICULAR_OFFSET);
                }));

    tab.addButton("Reset Pose To Vision")
        .setupOnPressCommand(
            Commands.runOnce(
                () -> {
                  drive.resetOdometry(RobotState.getVisionPose());
                },
                drive));
  }

  private void buildClimberTab() {
    String key = "Climber";
    ElasticTab tab = new ElasticTab(key);
    elasticTabMap.put(key, tab);

    tab.addButton("Climber Out (While Held)")
        .setupWhileHeldCommand(climber.climbVoltOut(), climber.climbSTOP());
    tab.addButton("Climber Deploy (When Pressed)").setupOnPressCommand(climber.climbDeploy());
    tab.addButton("Climber Full Deploy (When Pressed)")
        .setupOnPressCommand(
            Commands.runOnce(() -> climbRoller.setClimbing(true))
                .andThen(
                    climbRoller
                        .holdCage()
                        .alongWith(intake.setIntakeStateCommand(IntakeState.IDLE))
                        .alongWith(
                            superstructure.setStateCommand(SuperstructureState.CLIMB, "Climb")))
                .andThen(new WaitCommand(0.25))
                .andThen(climber.climbDeploy())
                .withName("climbDeployButton"));
    tab.addButton("Climber Climb (When Pressed)").setupOnPressCommand(climber.climbClimb());
    tab.addButton("Run Climb Rollers (When Pressed)")
        .setupWhileHeldCommand(climbRoller.holdCage(), climbRoller.rollerSTOP());
  }

  private void buildLedTab() {
    String key = "Led";
    ElasticTab tab = new ElasticTab(key);
    elasticTabMap.put(key, tab);

    tab.addButton("Solid Red")
        .setupOnPressCommandIgnoringDisabled(led.commandSolidColor(LedState.kRed));
    tab.addButton("Right Red")
        .setupOnPressCommandIgnoringDisabled(led.commandSolidColor(LedState.kRed, LedStrip.RIGHT));
    tab.addButton("Left Yellow")
        .setupOnPressCommandIgnoringDisabled(
            led.commandSolidColor(LedState.kYellow, LedStrip.LEFT));
    tab.addButton("Right Red Left Yellow")
        .setupOnPressCommandIgnoringDisabled(
            led.commandSolidColor(LedState.kRed, LedStrip.RIGHT)
                .andThen(led.commandSolidColor(LedState.kYellow, LedStrip.LEFT)));
    tab.addButton("Solid Orange").setupOnPressCommandIgnoringDisabled(led.commandSetOrange());
    tab.addButton("Solid Teal").setupOnPressCommandIgnoringDisabled(led.commandSetTeal());
    tab.addButton("Blink Red")
        .setupOnPressCommandIgnoringDisabled(led.commandBlinkingState(LedState.kRed, 0.5));
    tab.addButton("Fire").setupOnPressCommandIgnoringDisabled(led.commandFire());
    tab.addButton("Rainbow").setupOnPressCommandIgnoringDisabled(led.commandRainbow());
    tab.addButton("ColorflowCO").setupOnPressCommandIgnoringDisabled(led.commandColorflowCO());
    tab.addButton("Off").setupOnPressCommandIgnoringDisabled(led.commandOff());
    tab.addButton("Half Orange")
        .setupOnPressCommandIgnoringDisabled(
            led.commandPercentageFull(() -> 0.5, LedState.kCOOrangeLed));
    tab.addButton("Partial Orange")
        .setupOnPressCommandIgnoringDisabled(
            led.commandSolidColorNumLeds(LedState.kCOOrangeLed, led::getLedsOn));
    tab.addButton("Larson Blue")
        .setupOnPressCommandIgnoringDisabled(led.commandLarson(LedState.kBlue));
    tab.addButton("Larson Red")
        .setupOnPressCommandIgnoringDisabled(led.commandLarson(LedState.kRed));
    tab.addButton("Twinkle Off Blue")
        .setupOnPressCommandIgnoringDisabled(led.commandTwinkle(LedState.kBlue, true));
    tab.addButton("Twinkle Off Red")
        .setupOnPressCommandIgnoringDisabled(led.commandTwinkle(LedState.kRed, true));
    tab.addButton("Twinkle Blue")
        .setupOnPressCommandIgnoringDisabled(led.commandTwinkle(LedState.kBlue, false));
    tab.addButton("Twinkle Red")
        .setupOnPressCommandIgnoringDisabled(led.commandTwinkle(LedState.kRed, false));
  }

  private void buildTestTab() {
    String key = "Test";
    ElasticTab tab = new ElasticTab(key);
    elasticTabMap.put(key, tab);

    Command dejamCommand =
        intake
            .setIntakeStateCommand(IntakeState.REJECT_CORAL)
            .andThen(new WaitCommand(0.2))
            .andThen(intake.setIntakeStateCommand(IntakeState.INTAKE));

    tab.addButton("Dejam Mode").setupOnPressCommand(dejamCommand);
  }
}
