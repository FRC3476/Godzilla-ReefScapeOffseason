// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EndEffectorConstants.ClawState;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Field.FieldConstants;
import frc.robot.Field.FieldUtils;
import frc.robot.RobotState.AlgaeIntake;
import frc.robot.RobotState.CoralBranch;
import frc.robot.RobotState.ReefSide;
import frc.robot.RobotState.ScoreLevel;
import frc.robot.RobotState.ScorePosition;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPosePIDCommand;
import frc.robot.commands.MagicDriveToPoseCommand;
import frc.robot.commands.PathfindToPoseCommand;
import frc.robot.commands.Score;
import frc.robot.commands.test.CleaningTest;
import frc.robot.subsystems.climb.Climber;
import frc.robot.subsystems.climb.ClimberIO;
import frc.robot.subsystems.climb.ClimberIOReal;
import frc.robot.subsystems.climb.ClimberIOSim;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOHardware;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.end_effector.Claw;
import frc.robot.subsystems.end_effector.ClawIO;
import frc.robot.subsystems.end_effector.ClawIOReal;
import frc.robot.subsystems.end_effector.ClawIOSim;
import frc.robot.subsystems.end_effector.EndEffector;
import frc.robot.subsystems.end_effector.EndEffectorIO;
import frc.robot.subsystems.end_effector.EndEffectorIOReal;
import frc.robot.subsystems.end_effector.EndEffectorIOSim;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOReal;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.led.LedState;
import frc.robot.subsystems.superstructure.CoralStateTracker;
import frc.robot.subsystems.superstructure.CoralStateTracker.CoralPosition;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionFieldPoseEstimate;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOHardwareLimelight;
import frc.robot.subsystems.vision.VisionIOSimPhoton;
import frc.robot.util.Controls.StreamDeck;
import frc.robot.util.Controls.StreamDeckButton;
import frc.robot.util.Controls.StreamDeckButtonConfig;
import frc.robot.util.PoseUtils;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final DriveSubsystem drive;
  private final Intake intake;
  private final EndEffector endEffector;
  private final Claw claw;
  private final Elevator elevator;
  private final Superstructure superstructure;
  private final Climber climber;
  private final Feeder feeder;
  private final Vision vision;

  private final Consumer<VisionFieldPoseEstimate> visionEstimateConsumer =
      new Consumer<VisionFieldPoseEstimate>() {
        @Override
        public void accept(VisionFieldPoseEstimate estimate) {
          drive.addVisionMeasurement(estimate);
        }
      };

  private final RobotState robotState = new RobotState(visionEstimateConsumer);

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final StreamDeck streamdeck = new StreamDeck();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        feeder = new Feeder(new FeederIOReal());
        intake = new Intake(new IntakeIOReal(), feeder);
        endEffector = new EndEffector(new EndEffectorIOReal());
        claw = new Claw(new ClawIOReal() {});
        elevator = new Elevator(new ElevatorIOReal());
        superstructure = new Superstructure(elevator, endEffector, this);
        climber = new Climber(new ClimberIOReal());
        vision = new Vision(new VisionIOHardwareLimelight(), robotState);
        drive =
            new DriveSubsystem(
                new DriveIOHardware(
                    robotState,
                    Constants.DriveConstants.kDrivetrain.getDriveTrainConstants(),
                    Constants.DriveConstants.kDrivetrain.getModuleConstants()),
                robotState);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        feeder = new Feeder(new FeederIOSim());
        intake = new Intake(new IntakeIOSim(), feeder);
        endEffector = new EndEffector(new EndEffectorIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        claw = new Claw(new ClawIOSim() {});
        superstructure = new Superstructure(elevator, endEffector, this);
        climber = new Climber(new ClimberIOSim());
        vision = new Vision(new VisionIOSimPhoton(), robotState);
        drive =
            new DriveSubsystem(
                new DriveIOSim(
                    robotState,
                    Constants.DriveConstants.kDrivetrain.getDriveTrainConstants(),
                    Constants.DriveConstants.kDrivetrain.getModuleConstants()),
                robotState);
        break;

      default:
        // Replayed robot, disable IO implementations
        feeder = new Feeder(new FeederIO() {});
        intake = new Intake(new IntakeIO() {}, feeder);
        endEffector = new EndEffector(new EndEffectorIO() {});
        claw = new Claw(new ClawIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        superstructure = new Superstructure(elevator, endEffector, this);
        climber = new Climber(new ClimberIO() {});
        vision = new Vision(new VisionIO() {}, robotState);
        drive = new DriveSubsystem(new DriveIO() {}, robotState);
        break;
    }

    // ====================LOADING COMMANDS====================
    NamedCommands.registerCommand(
        "SuperStructureStartup",
        superstructure.setStateCommand(SuperstructureState.STOW, "STOW").asProxy());

    NamedCommands.registerCommand(
        "IntakeStartup", intake.setIntakeStateCommand(IntakeState.IDLE).asProxy());

    NamedCommands.registerCommand(
        "ScoreTargetStartup",
        new InstantCommand(() -> robotState.getStoredScorePosition().setScoreLevel(ScoreLevel.L4)));

    // ====================SCORING COMMANDS====================
    NamedCommands.registerCommand(
        "AimL4", superstructure.setStateCommand(SuperstructureState.L4_AIM, "L4 AIM").asProxy());

    // LEFT ALIGN
    NamedCommands.registerCommand(
        "FinalLeftPoleAlign",
        new MagicDriveToPoseCommand(
                drive,
                () ->
                    PoseUtils.getPerpendicularOffsetPose(
                        FieldUtils.getClosestReef().leftPole.getPose(),
                        DriveConstants.AUTO_ALIGN_PERPENDICULAR_OFFSET))
            .withTimeout(2.0));

    // RIGHT ALIGN
    NamedCommands.registerCommand(
        "FinalRightPoleAlign",
        new MagicDriveToPoseCommand(
                drive,
                () ->
                    PoseUtils.getPerpendicularOffsetPose(
                        FieldUtils.getClosestReef().rightPole.getPose(),
                        DriveConstants.AUTO_ALIGN_PERPENDICULAR_OFFSET))
            .withTimeout(2.0));

    NamedCommands.registerCommand("ConfirmScore", new Score(superstructure, claw, robotState));

    NamedCommands.registerCommand(
        "StowRobotState",
        new WaitUntilCommand(() -> RobotState.isSafeToStow())
            .andThen(superstructure.setStateCommand(SuperstructureState.STOW, "STOW").asProxy()));

    // ====================INTAKE COMMANDS====================
    NamedCommands.registerCommand(
        "IntakeEnable", intake.setIntakeStateCommand(IntakeState.INTAKE).asProxy());

    NamedCommands.registerCommand(
        "DriveToCoral", DriveCommands.driveToCoral(drive, vision).withTimeout(3.0));

    // ====================CORAL TRACKING COMMANDS====================
    NamedCommands.registerCommand(
        "SeesCoral", new WaitUntilCommand(() -> vision.isCoralDetected()).withTimeout(3.0));
    NamedCommands.registerCommand(
        "IsCoralInFeeder",
        new WaitUntilCommand(
            () -> CoralStateTracker.getCurrentPosition() == CoralPosition.AT_FEEDER));
    NamedCommands.registerCommand(
        "IsCoralInEndEffector",
        new WaitUntilCommand(
            () -> CoralStateTracker.getCurrentPosition() == CoralPosition.STAGED_IN_END_EFFECTOR));

    // ====================PID DRIVE COMMANDS====================
    // NamedCommands.registerCommand("DriveStraightRed", new DriveToPosePIDCommand(drive, () -> new
    // Pose2d(6.043, 4.060, Rotation2d.k180deg)));
    // NamedCommands.registerCommand("DriveStraightBlue", new DriveToPosePIDCommand(PEND, PEND,
    // PEND));

    // ============================================================================================================================================

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Slip Current Characterization (Wall Test)",
    //     DriveCommands.slipCurrentCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // autoChooser.addOption("Drivetrain Test", new DrivetrainTest(drive));

    // Configure default commands for subsystems
    RegisterDefaultCommands();

    // Build elastic tabs for testing
    buildElasticTabs();

    // Configure the button bindings
    configureButtonBindings();

    // Configure arbitrary triggers
    configureArbitraryTriggers();

    configureSuperstructureTrigger();
  }

  private void configureButtonBindings() {
    configureXboxBindings();
    // configureTestingStreamDeckBindings();
    configureDriveStreamDeckBindings();
  }

  private void RegisterDefaultCommands() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX() * Math.abs(controller.getRightX())));
    // elevator.setDefaultCommand(defaultElevatorCommand());
    // endEffector.setDefaultCommand(defaultEndEffectorCommand());
    claw.setDefaultCommand(claw.clawDefault());
    intake.setDefaultCommand(intake.intakeDefault());
  }

  private void buildElasticTabs() {
    buildIntakeTab();
    buildEndEffectorTab();
    buildElevatorTab();
    buildSuperstructureTab();
    buildClimberTab();
    buildDriveTab();
    buildTestTab();
  }

  private void buildIntakeTab() {
    // Get the NetworkTable for the Intake tab
    NetworkTable intakeTable = NetworkTableInstance.getDefault().getTable("Elastic/Intake");

    // Create NetworkTableEntry instances for while-held functionality
    NetworkTableEntry intakeForwardEntry = intakeTable.getEntry("Roller Forward (While Held)");
    NetworkTableEntry intakeReverseEntry = intakeTable.getEntry("Roller Reverse (While Held)");

    NetworkTableEntry intakeUpEntry = intakeTable.getEntry("Pivot Up (While Held)");
    NetworkTableEntry intakeDownEntry = intakeTable.getEntry("Pivot Down (While Held)");

    NetworkTableEntry intakeUpPosEntry = intakeTable.getEntry("Pivot Up (When Pressed)");
    NetworkTableEntry intakeDownPosEntry =
        intakeTable.getEntry("Pivot Intake Position (When Pressed)");
    NetworkTableEntry intakeScoringPosEntry =
        intakeTable.getEntry("Pivot Scoring Position (When Pressed)");
    NetworkTableEntry intakeZeroPosEntry =
        intakeTable.getEntry("Pivot Zero Position (When Pressed)");

    NetworkTableEntry l1BarUpEntry = intakeTable.getEntry("L1 Bar Up (While Held)");
    NetworkTableEntry l1BarDownEntry = intakeTable.getEntry("L1 Bar Down (While Held)");

    NetworkTableEntry feederForwardEntry = intakeTable.getEntry("Feeder In (While Held)");
    NetworkTableEntry feederReverseEntry = intakeTable.getEntry("Feeder Out (While Held)");

    // Intake State buttons
    NetworkTableEntry intakeStateStowEntry = intakeTable.getEntry("STOW (When Pressed)");
    NetworkTableEntry intakeStateIntakeL1Entry = intakeTable.getEntry("IntakeL1 (When Pressed)");
    NetworkTableEntry intakeStateIntakeEntry = intakeTable.getEntry("Intake (When Pressed)");
    NetworkTableEntry intakeStateRejectCoralEntry =
        intakeTable.getEntry("Reject Coral (When Pressed)");
    NetworkTableEntry intakeStateIdleEntry = intakeTable.getEntry("Idle (When Pressed)");
    NetworkTableEntry intakeStateHandOffEntry = intakeTable.getEntry("Hand Off (When Pressed)");
    NetworkTableEntry intakeStateScoringEntry = intakeTable.getEntry("Scoring (When Pressed)");
    NetworkTableEntry intakeStateScoringPrepEntry =
        intakeTable.getEntry("Scoring Prep (When Pressed)");
    NetworkTableEntry intakeStateNoneEntry = intakeTable.getEntry("Manual Control (When Pressed)");

    // Initialize entries with default values
    intakeForwardEntry.setBoolean(false);
    intakeReverseEntry.setBoolean(false);

    intakeUpEntry.setBoolean(false);
    intakeDownEntry.setBoolean(false);

    intakeUpPosEntry.setBoolean(false);
    intakeDownPosEntry.setBoolean(false);
    intakeScoringPosEntry.setBoolean(false);
    intakeZeroPosEntry.setBoolean(false);

    l1BarUpEntry.setBoolean(false);
    l1BarDownEntry.setBoolean(false);

    feederForwardEntry.setBoolean(false);
    feederReverseEntry.setBoolean(false);

    // Initialize intake state entries
    intakeStateStowEntry.setBoolean(false);
    intakeStateIntakeL1Entry.setBoolean(false);
    intakeStateIntakeEntry.setBoolean(false);
    intakeStateRejectCoralEntry.setBoolean(false);
    intakeStateIdleEntry.setBoolean(false);
    intakeStateHandOffEntry.setBoolean(false);
    intakeStateScoringEntry.setBoolean(false);
    intakeStateScoringPrepEntry.setBoolean(false);
    intakeStateNoneEntry.setBoolean(false);

    // Create triggers based on the NetworkTableEntry values
    Trigger intakeForwardTrigger = new Trigger(() -> intakeForwardEntry.getBoolean(false));
    Trigger intakeReverseTrigger = new Trigger(() -> intakeReverseEntry.getBoolean(false));

    Trigger intakeUpTrigger = new Trigger(() -> intakeUpEntry.getBoolean(false));
    Trigger intakeDownTrigger = new Trigger(() -> intakeDownEntry.getBoolean(false));

    Trigger intakeUpPosTrigger = new Trigger(() -> intakeUpPosEntry.getBoolean(false));
    Trigger intakeDownPosTrigger = new Trigger(() -> intakeDownPosEntry.getBoolean(false));
    Trigger intakeScoringPosTrigger = new Trigger(() -> intakeScoringPosEntry.getBoolean(false));
    Trigger intakeZeroPosTrigger = new Trigger(() -> intakeZeroPosEntry.getBoolean(false));

    Trigger feederInTrigger = new Trigger(() -> feederForwardEntry.getBoolean(false));
    Trigger feederOutTrigger = new Trigger(() -> feederReverseEntry.getBoolean(false));

    // Create triggers for intake state buttons
    Trigger intakeStateStowTrigger = new Trigger(() -> intakeStateStowEntry.getBoolean(false));

    Trigger intakeStateIntakeTrigger = new Trigger(() -> intakeStateIntakeEntry.getBoolean(false));
    Trigger intakeStateRejectCoralTrigger =
        new Trigger(() -> intakeStateRejectCoralEntry.getBoolean(false));
    Trigger intakeStateIdleTrigger = new Trigger(() -> intakeStateIdleEntry.getBoolean(false));
    Trigger intakeStateHandOffTrigger =
        new Trigger(() -> intakeStateHandOffEntry.getBoolean(false));
    Trigger intakeStateScoringTrigger =
        new Trigger(() -> intakeStateScoringEntry.getBoolean(false));
    Trigger intakeStateScoringPrepTrigger =
        new Trigger(() -> intakeStateScoringPrepEntry.getBoolean(false));
    Trigger intakeStateNoneTrigger = new Trigger(() -> intakeStateNoneEntry.getBoolean(false));

    // Configure the while-held behavior
    intakeForwardTrigger.whileTrue(intake.intakeFWD());
    intakeForwardTrigger.onFalse(intake.intakeSTOP());

    intakeReverseTrigger.whileTrue(intake.intakeRVS());
    intakeReverseTrigger.onFalse(intake.intakeSTOP());

    intakeUpTrigger.whileTrue(intake.pivotManualTestForward());
    intakeUpTrigger.onFalse(intake.pivotStop());

    intakeDownTrigger.whileTrue(intake.pivotManualTestReverse());
    intakeDownTrigger.onFalse(intake.pivotStop());

    feederInTrigger.whileTrue(intake.feederFWD());
    feederInTrigger.onFalse(intake.feederSTOP());

    feederOutTrigger.whileTrue(intake.feederRVS());
    feederOutTrigger.onFalse(intake.feederSTOP());

    intakeUpPosTrigger.onTrue(
        intake.setPivotUp().andThen(() -> intakeUpPosEntry.setBoolean(false)));
    intakeDownPosTrigger.onTrue(
        intake.setPivotDown().andThen(() -> intakeDownPosEntry.setBoolean(false)));
    intakeScoringPosTrigger.onTrue(
        intake.setPivotScoring().andThen(() -> intakeScoringPosEntry.setBoolean(false)));
    intakeZeroPosTrigger.onTrue(
        intake.zeroPivotAtPivotUp().andThen(() -> intakeZeroPosEntry.setBoolean(false)));

    // Configure intake state button triggers
    intakeStateStowTrigger.onTrue(
        intake
            .setIntakeStateCommand(IntakeState.STOW)
            .andThen(() -> intakeStateStowEntry.setBoolean(false)));
    intakeStateIntakeTrigger.onTrue(
        intake
            .setIntakeStateCommand(IntakeState.INTAKE)
            .andThen(() -> intakeStateIntakeEntry.setBoolean(false)));
    intakeStateRejectCoralTrigger.onTrue(
        intake
            .setIntakeStateCommand(IntakeState.REJECT_CORAL)
            .andThen(() -> intakeStateRejectCoralEntry.setBoolean(false)));
    intakeStateIdleTrigger.onTrue(
        intake
            .setIntakeStateCommand(IntakeState.IDLE)
            .andThen(() -> intakeStateIdleEntry.setBoolean(false)));
    intakeStateHandOffTrigger.onTrue(
        intake
            .setIntakeStateCommand(IntakeState.HAND_OFF)
            .andThen(() -> intakeStateHandOffEntry.setBoolean(false)));
    intakeStateScoringTrigger.onTrue(
        intake
            .setIntakeStateCommand(IntakeState.SCORING)
            .andThen(() -> intakeStateScoringEntry.setBoolean(false)));
    intakeStateScoringPrepTrigger.onTrue(
        intake
            .setIntakeStateCommand(IntakeState.SCORING_PREP)
            .andThen(() -> intakeStateScoringPrepEntry.setBoolean(false)));
    intakeStateNoneTrigger.onTrue(
        intake
            .setIntakeStateCommand(IntakeState.NONE)
            .andThen(() -> intakeStateNoneEntry.setBoolean(false)));
  }

  private void buildEndEffectorTab() {
    // Get the NetworkTable for the EndEffector tab
    NetworkTable endEffectorTable =
        NetworkTableInstance.getDefault().getTable("Elastic/EndEffector");

    // Create NetworkTableEntry instances for while-held functionality
    NetworkTableEntry clawForwardEntry = endEffectorTable.getEntry("Roller Forward (While Held)");
    NetworkTableEntry clawReverseEntry = endEffectorTable.getEntry("Roller Reverse (While Held)");
    NetworkTableEntry clawHoldEntry = endEffectorTable.getEntry("Roller Hold (When Pressed)");

    NetworkTableEntry clawScoreEntry = endEffectorTable.getEntry("Claw Score");
    NetworkTableEntry clawScoreL1Entry = endEffectorTable.getEntry("Claw Score L1");
    NetworkTableEntry clawAlgaeEntry = endEffectorTable.getEntry("Claw Algae Hold");
    NetworkTableEntry clawNoneEntry = endEffectorTable.getEntry("Claw None");

    NetworkTableEntry pivotUpEntry = endEffectorTable.getEntry("Pivot Up (While Held)");
    NetworkTableEntry pivotDownEntry = endEffectorTable.getEntry("Pivot Down (While Held)");

    NetworkTableEntry pivotSafeUpEntry = endEffectorTable.getEntry("Pivot Safe Up (When Pressed)");
    NetworkTableEntry pivotUpPosEntry = endEffectorTable.getEntry("Pivot Fully Up (When Pressed)");
    NetworkTableEntry pivotSafeDownPosEntry =
        endEffectorTable.getEntry("Pivot Safe Down (When Pressed)");
    NetworkTableEntry pivotDownPosEntry =
        endEffectorTable.getEntry("Pivot Fully Down (When Pressed)");
    NetworkTableEntry pivotMiddlePosEntry =
        endEffectorTable.getEntry("Pivot Middle (When Pressed)");
    NetworkTableEntry pivotManualZeroEntry = endEffectorTable.getEntry("Pivot Zero (When Pressed)");

    // Initialize entries with default values
    clawForwardEntry.setBoolean(false);
    clawReverseEntry.setBoolean(false);
    clawHoldEntry.setBoolean(false);
    clawScoreEntry.setBoolean(false);
    clawScoreL1Entry.setBoolean(false);
    clawAlgaeEntry.setBoolean(false);
    clawNoneEntry.setBoolean(false);

    pivotUpEntry.setBoolean(false);
    pivotDownEntry.setBoolean(false);

    pivotSafeUpEntry.setBoolean(false);
    pivotUpPosEntry.setBoolean(false);
    pivotSafeDownPosEntry.setBoolean(false);
    pivotDownPosEntry.setBoolean(false);
    pivotMiddlePosEntry.setBoolean(false);

    pivotManualZeroEntry.setBoolean(false);

    // Create triggers based on the NetworkTableEntry values
    Trigger clawForwardTrigger = new Trigger(() -> clawForwardEntry.getBoolean(false));
    Trigger clawReverseTrigger = new Trigger(() -> clawReverseEntry.getBoolean(false));
    Trigger clawHoldTrigger = new Trigger(() -> clawHoldEntry.getBoolean(false));
    Trigger clawScoreTrigger = new Trigger(() -> clawScoreEntry.getBoolean(false));
    Trigger clawScoreL1Trigger = new Trigger(() -> clawScoreL1Entry.getBoolean(false));
    Trigger clawAlgaeTrigger = new Trigger(() -> clawAlgaeEntry.getBoolean(false));
    Trigger clawNoneTrigger = new Trigger(() -> clawNoneEntry.getBoolean(false));

    Trigger pivotUpTrigger = new Trigger(() -> pivotUpEntry.getBoolean(false));
    Trigger pivotDownTrigger = new Trigger(() -> pivotDownEntry.getBoolean(false));

    Trigger pivotUpPosTrigger = new Trigger(() -> pivotUpPosEntry.getBoolean(false));
    Trigger pivotSafeUpPosTrigger = new Trigger(() -> pivotSafeUpEntry.getBoolean(false));
    Trigger pivotDownPosTrigger = new Trigger(() -> pivotDownPosEntry.getBoolean(false));
    Trigger pivotSafeDownPosTrigger = new Trigger(() -> pivotSafeDownPosEntry.getBoolean(false));
    Trigger pivotMiddlePosTrigger = new Trigger(() -> pivotMiddlePosEntry.getBoolean(false));

    Trigger pivotManualZeroTrigger = new Trigger(() -> pivotManualZeroEntry.getBoolean(false));

    // Configure the while-held behavior
    clawForwardTrigger.whileTrue(claw.rollerFWD());
    clawForwardTrigger.onFalse(claw.rollerSTOP());

    clawReverseTrigger.whileTrue(claw.rollerRVS());
    clawReverseTrigger.onFalse(claw.rollerSTOP());

    clawHoldTrigger.onTrue(claw.holdAlgae());
    clawHoldTrigger.onFalse(claw.holdAlgae());

    clawScoreTrigger.onTrue(
        claw.setClawStateCommand(ClawState.SCORING)
            .andThen(() -> clawScoreEntry.setBoolean(false)));

    clawScoreL1Trigger.onTrue(
        claw.setClawStateCommand(ClawState.SCORING_L1)
            .andThen(() -> clawScoreL1Entry.setBoolean(false)));

    clawAlgaeTrigger.onTrue(
        claw.setClawStateCommand(ClawState.ALGAE).andThen(() -> clawAlgaeEntry.setBoolean(false)));

    clawNoneTrigger.onTrue(
        claw.setClawStateCommand(ClawState.NONE).andThen(() -> clawNoneEntry.setBoolean(false)));

    pivotUpTrigger.whileTrue(endEffector.pivotUP());
    pivotUpTrigger.onFalse(endEffector.pivotSTOP());

    pivotDownTrigger.whileTrue(endEffector.pivotDOWN());
    pivotDownTrigger.onFalse(endEffector.pivotSTOP());

    pivotUpPosTrigger.onTrue(
        endEffector
            .rotatePivotCommand(() -> Constants.EndEffectorConstants.MAX_ANGLE_ROTATIONS)
            .andThen(() -> pivotUpPosEntry.setBoolean(false)));
    pivotSafeUpPosTrigger.onTrue(
        endEffector
            .rotatePivotCommand(() -> Constants.EndEffectorConstants.MAX_SAFE_ANGLE_ROTATIONS)
            .andThen(() -> pivotSafeUpEntry.setBoolean(false)));
    pivotSafeDownPosTrigger.onTrue(
        endEffector
            .rotatePivotCommand(() -> Constants.EndEffectorConstants.MIN_SAFE_ANGLE_ROTATIONS)
            .andThen(() -> pivotSafeDownPosEntry.setBoolean(false)));
    pivotDownPosTrigger.onTrue(
        endEffector
            .rotatePivotCommand(() -> Constants.EndEffectorConstants.MIN_ANGLE_ROTATIONS)
            .andThen(() -> pivotDownPosEntry.setBoolean(false)));
    pivotMiddlePosTrigger.onTrue(
        endEffector
            .rotatePivotCommand(
                () ->
                    (Constants.EndEffectorConstants.MIN_SAFE_ANGLE_ROTATIONS
                            + Constants.EndEffectorConstants.MAX_SAFE_ANGLE_ROTATIONS)
                        / 2)
            .andThen(() -> pivotMiddlePosEntry.setBoolean(false)));
    pivotManualZeroTrigger.onTrue(
        endEffector.setPivotZero().andThen(() -> pivotManualZeroEntry.setBoolean(false)));
  }

  private void buildElevatorTab() {
    // Get the NetworkTable for the Elevator tab
    NetworkTable elevatorTable = NetworkTableInstance.getDefault().getTable("Elastic/Elevator");

    // Create NetworkTableEntry instances for while-held functionality
    NetworkTableEntry elevatorUpEntry = elevatorTable.getEntry("Elevator Up (While Held)");
    NetworkTableEntry elevatorDownEntry = elevatorTable.getEntry("Elevator Down (While Held)");

    NetworkTableEntry elevatorL2Entry = elevatorTable.getEntry("Elevator L2 (When Pressed)");
    NetworkTableEntry elevatorL3Entry = elevatorTable.getEntry("Elevator L3 (When Pressed)");
    NetworkTableEntry elevatorL4Entry = elevatorTable.getEntry("Elevator L4 (When Pressed)");
    NetworkTableEntry elevatorDownPosEntry =
        elevatorTable.getEntry("Elevator Down Pos (When Pressed)");
    NetworkTableEntry elevatorManualZeroEntry =
        elevatorTable.getEntry("Zero the Elevator (When Pressed)");

    // Initialize entries with default values
    elevatorUpEntry.setBoolean(false);
    elevatorDownEntry.setBoolean(false);
    elevatorL2Entry.setBoolean(false);
    elevatorL3Entry.setBoolean(false);
    elevatorL4Entry.setBoolean(false);
    elevatorDownPosEntry.setBoolean(false);
    elevatorManualZeroEntry.setBoolean(false);

    // Create triggers based on the NetworkTableEntry values
    Trigger elevatorUpTrigger = new Trigger(() -> elevatorUpEntry.getBoolean(false));
    Trigger elevatorDownTrigger = new Trigger(() -> elevatorDownEntry.getBoolean(false));
    Trigger elevatorL2Trigger = new Trigger(() -> elevatorL2Entry.getBoolean(false));
    Trigger elevatorL3Trigger = new Trigger(() -> elevatorL3Entry.getBoolean(false));
    Trigger elevatorL4Trigger = new Trigger(() -> elevatorL4Entry.getBoolean(false));
    Trigger elevatorDownPosTrigger = new Trigger(() -> elevatorDownPosEntry.getBoolean(false));
    Trigger elevatorManualZeroTrigger =
        new Trigger(() -> elevatorManualZeroEntry.getBoolean(false));

    // Configure the while-held behavior
    elevatorUpTrigger.whileTrue(elevator.elevatorUP());
    elevatorUpTrigger.onFalse(elevator.elevatorSTOP());

    elevatorDownTrigger.whileTrue(elevator.elevatorDWN());
    elevatorDownTrigger.onFalse(elevator.elevatorSTOP());

    elevatorL2Trigger.onTrue(
        elevator
            .setTargetPositionCommand(() -> SuperstructureState.L2_AIM.getElevatorHeight())
            .andThen(() -> elevatorL2Entry.setBoolean(false)));
    elevatorL3Trigger.onTrue(
        elevator
            .setTargetPositionCommand(() -> SuperstructureState.L3_AIM.getElevatorHeight())
            .andThen(() -> elevatorL3Entry.setBoolean(false)));
    elevatorL4Trigger.onTrue(
        elevator
            .setTargetPositionCommand(() -> SuperstructureState.L4_AIM.getElevatorHeight())
            .andThen(() -> elevatorL4Entry.setBoolean(false)));
    elevatorDownPosTrigger.onTrue(
        elevator
            .setTargetPositionCommand(() -> SuperstructureState.STOW.getElevatorHeight())
            .andThen(() -> elevatorDownPosEntry.setBoolean(false)));
    elevatorManualZeroTrigger.onTrue(
        elevator.manualSetElevatorZero().andThen(() -> elevatorManualZeroEntry.setBoolean(false)));
  }

  private void buildSuperstructureTab() {
    superstructure.setStateCommand(SuperstructureState.STOW, "Set STOW");
    // Get the NetworkTable for the Superstructure tab
    NetworkTable superstructureTable =
        NetworkTableInstance.getDefault().getTable("Elastic/Superstructure");

    // Create NetworkTableEntry instances for each SuperstructureState
    NetworkTableEntry stowEntry = superstructureTable.getEntry("STOW");
    NetworkTableEntry stowCoralEntry = superstructureTable.getEntry("STOW_CORAL");
    NetworkTableEntry stowAlgaeEntry = superstructureTable.getEntry("STOW_ALGAE");
    NetworkTableEntry intakeCoralEntry = superstructureTable.getEntry("INTAKE_CORAL");
    NetworkTableEntry intakeCoralL1Entry = superstructureTable.getEntry("INTAKE_CORAL_L1");
    NetworkTableEntry feedEntry = superstructureTable.getEntry("FEED");
    NetworkTableEntry l1PivotEntry = superstructureTable.getEntry("L1_PIVOT");
    NetworkTableEntry l2FadeawayEntry = superstructureTable.getEntry("L2_FADEAWAY");
    NetworkTableEntry l3FadeawayEntry = superstructureTable.getEntry("L3_FADEAWAY");
    NetworkTableEntry l4FadeawayEntry = superstructureTable.getEntry("L4_FADEAWAY");
    NetworkTableEntry l1ScoreEntry = superstructureTable.getEntry("L1_SCORE");
    NetworkTableEntry l2ScoreEntry = superstructureTable.getEntry("L2_SCORE");
    NetworkTableEntry l3ScoreEntry = superstructureTable.getEntry("L3_SCORE");
    NetworkTableEntry l4ScoreEntry = superstructureTable.getEntry("L4_SCORE");
    NetworkTableEntry l2AwayFromReefEntry = superstructureTable.getEntry("L2_AWAY_FROM_REEF");
    NetworkTableEntry l3AwayFromReefEntry = superstructureTable.getEntry("L3_AWAY_FROM_REEF");
    NetworkTableEntry l4AwayFromReefEntry = superstructureTable.getEntry("L4_AWAY_FROM_REEF");
    NetworkTableEntry algaeHighIntakeEntry = superstructureTable.getEntry("ALGAE_HIGH_INTAKE");
    NetworkTableEntry algaeLowIntakeEntry = superstructureTable.getEntry("ALGAE_LOW_INTAKE");
    NetworkTableEntry processorAimEntry = superstructureTable.getEntry("PROCESSOR_AIM");
    NetworkTableEntry bargeAimCenterEntry = superstructureTable.getEntry("BARGE_AIM_CENTER");
    NetworkTableEntry bargeAimBackwardEntry = superstructureTable.getEntry("BARGE_AIM_BACKWARD");
    NetworkTableEntry bargeAimForwardEntry = superstructureTable.getEntry("BARGE_AIM_FORWARD");

    // Initialize entries with default values
    stowEntry.setBoolean(false);
    stowCoralEntry.setBoolean(false);
    stowAlgaeEntry.setBoolean(false);
    intakeCoralEntry.setBoolean(false);
    intakeCoralL1Entry.setBoolean(false);
    feedEntry.setBoolean(false);
    l1PivotEntry.setBoolean(false);
    l2FadeawayEntry.setBoolean(false);
    l3FadeawayEntry.setBoolean(false);
    l4FadeawayEntry.setBoolean(false);
    l1ScoreEntry.setBoolean(false);
    l2ScoreEntry.setBoolean(false);
    l3ScoreEntry.setBoolean(false);
    l4ScoreEntry.setBoolean(false);
    l2AwayFromReefEntry.setBoolean(false);
    l3AwayFromReefEntry.setBoolean(false);
    l4AwayFromReefEntry.setBoolean(false);
    algaeHighIntakeEntry.setBoolean(false);
    algaeLowIntakeEntry.setBoolean(false);
    processorAimEntry.setBoolean(false);
    bargeAimCenterEntry.setBoolean(false);
    bargeAimBackwardEntry.setBoolean(false);
    bargeAimForwardEntry.setBoolean(false);

    // Create triggers for each button
    Trigger stowTrigger = new Trigger(() -> stowEntry.getBoolean(false));
    Trigger stowCoralTrigger = new Trigger(() -> stowCoralEntry.getBoolean(false));
    Trigger stowAlgaeTrigger = new Trigger(() -> stowAlgaeEntry.getBoolean(false));
    Trigger intakeCoralTrigger = new Trigger(() -> intakeCoralEntry.getBoolean(false));
    Trigger intakeCoralL1Trigger = new Trigger(() -> intakeCoralL1Entry.getBoolean(false));
    Trigger feedTrigger = new Trigger(() -> feedEntry.getBoolean(false));
    Trigger l1PivotTrigger = new Trigger(() -> l1PivotEntry.getBoolean(false));
    Trigger l2FadeawayTrigger = new Trigger(() -> l2FadeawayEntry.getBoolean(false));
    Trigger l3FadeawayTrigger = new Trigger(() -> l3FadeawayEntry.getBoolean(false));
    Trigger l4FadeawayTrigger = new Trigger(() -> l4FadeawayEntry.getBoolean(false));
    Trigger l1ScoreTrigger = new Trigger(() -> l1ScoreEntry.getBoolean(false));
    Trigger l2ScoreTrigger = new Trigger(() -> l2ScoreEntry.getBoolean(false));
    Trigger l3ScoreTrigger = new Trigger(() -> l3ScoreEntry.getBoolean(false));
    Trigger l4ScoreTrigger = new Trigger(() -> l4ScoreEntry.getBoolean(false));
    Trigger l2AwayFromReefTrigger = new Trigger(() -> l2AwayFromReefEntry.getBoolean(false));
    Trigger l3AwayFromReefTrigger = new Trigger(() -> l3AwayFromReefEntry.getBoolean(false));
    Trigger l4AwayFromReefTrigger = new Trigger(() -> l4AwayFromReefEntry.getBoolean(false));
    Trigger algaeHighIntakeTrigger = new Trigger(() -> algaeHighIntakeEntry.getBoolean(false));
    Trigger algaeLowIntakeTrigger = new Trigger(() -> algaeLowIntakeEntry.getBoolean(false));
    Trigger processorAimTrigger = new Trigger(() -> processorAimEntry.getBoolean(false));
    Trigger bargeAimCenterTrigger = new Trigger(() -> bargeAimCenterEntry.getBoolean(false));
    Trigger bargeAimBackwardTrigger = new Trigger(() -> bargeAimBackwardEntry.getBoolean(false));
    Trigger bargeAimForwardTrigger = new Trigger(() -> bargeAimForwardEntry.getBoolean(false));

    // Wire triggers to superstructure state commands
    stowTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.STOW, "Set STOW")
            .andThen(() -> stowEntry.setBoolean(false)));
    stowCoralTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.STOW_CORAL, "Set STOW_CORAL")
            .andThen(() -> stowCoralEntry.setBoolean(false)));
    stowAlgaeTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.STOW_ALGAE, "Set STOW_ALGAE")
            .andThen(() -> stowAlgaeEntry.setBoolean(false)));
    intakeCoralTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.INTAKE_CORAL, "Set INTAKE_CORAL")
            .andThen(() -> intakeCoralEntry.setBoolean(false)));
    intakeCoralL1Trigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.INTAKE_CORAL_L1, "Set INTAKE_CORAL_L1")
            .andThen(() -> intakeCoralL1Entry.setBoolean(false)));
    feedTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.FEED, "Set FEED")
            .andThen(() -> feedEntry.setBoolean(false)));
    l1PivotTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.L1_PIVOT, "Set L1_PIVOT")
            .andThen(() -> l1PivotEntry.setBoolean(false)));
    l2FadeawayTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.L2_FADEAWAY, "Set L2_FADEAWAY")
            .andThen(() -> l2FadeawayEntry.setBoolean(false)));
    l3FadeawayTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.L3_FADEAWAY, "Set L3_FADEAWAY")
            .andThen(() -> l3FadeawayEntry.setBoolean(false)));
    l4FadeawayTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.L4_FADEAWAY, "Set L4_FADEAWAY")
            .andThen(() -> l4FadeawayEntry.setBoolean(false)));
    l1ScoreTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.L1_PIVOT, "Set L1_SCORE")
            .andThen(() -> l1ScoreEntry.setBoolean(false)));
    l2ScoreTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.L2_AIM, "Set L2_SCORE")
            .andThen(() -> l2ScoreEntry.setBoolean(false)));
    l3ScoreTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.L3_AIM, "Set L3_SCORE")
            .andThen(() -> l3ScoreEntry.setBoolean(false)));
    l4ScoreTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.L4_AIM, "Set L4_SCORE")
            .andThen(() -> l4ScoreEntry.setBoolean(false)));
    l2AwayFromReefTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.L2_AWAY_FROM_REEF, "Set L2_SCORE")
            .andThen(() -> l2AwayFromReefEntry.setBoolean(false)));
    l3AwayFromReefTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.L3_AWAY_FROM_REEF, "Set L3_SCORE")
            .andThen(() -> l3AwayFromReefEntry.setBoolean(false)));
    l4AwayFromReefTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.L4_AWAY_FROM_REEF, "Set L4_SCORE")
            .andThen(() -> l4AwayFromReefEntry.setBoolean(false)));
    algaeHighIntakeTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.ALGAE_HIGH_INTAKE, "Set ALGAE_HIGH_INTAKE")
            .andThen(() -> algaeHighIntakeEntry.setBoolean(false)));
    algaeLowIntakeTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.ALGAE_LOW_INTAKE, "Set ALGAE_LOW_INTAKE")
            .andThen(() -> algaeLowIntakeEntry.setBoolean(false)));
    processorAimTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.PROCESSOR_AIM, "Set PROCESSOR_AIM")
            .andThen(() -> processorAimEntry.setBoolean(false)));
    bargeAimCenterTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.BARGE_AIM_CENTER, "Set BARGE_AIM_CENTER")
            .andThen(() -> bargeAimCenterEntry.setBoolean(false)));
    bargeAimBackwardTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.BARGE_AIM_BACKWARD, "Set BARGE_AIM_BACKWARD")
            .andThen(() -> bargeAimBackwardEntry.setBoolean(false)));
    bargeAimForwardTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.BARGE_AIM_FORWARD, "Set BARGE_AIM_FORWARD")
            .andThen(() -> bargeAimForwardEntry.setBoolean(false)));

    // Add current state and target state monitoring
    NetworkTableEntry currentStateEntry = superstructureTable.getEntry("Current State");
    NetworkTableEntry targetStateEntry = superstructureTable.getEntry("Target State");

    // Update these entries periodically in the periodic method of RobotContainer
    // For now, we'll create them so they appear on the dashboard
    currentStateEntry.setString("Unknown");
    targetStateEntry.setString("Unknown");
  }

  private void buildDriveTab() {
    NetworkTable driveTable = NetworkTableInstance.getDefault().getTable("Elastic/Drive");

    NetworkTableEntry driveFeedforwardEntry = driveTable.getEntry("Characterize Feedforward");
    NetworkTableEntry driveSlipCurrentEntry = driveTable.getEntry("Characterize Slip Current");
    NetworkTableEntry driveWheelRadiusEntry = driveTable.getEntry("Characterize Wheel Radius");
    NetworkTableEntry driveStopXEntry = driveTable.getEntry("Drive Stop X");
    NetworkTableEntry driveForwardEntry = driveTable.getEntry("Drive Forward");
    NetworkTableEntry driveClockwiseEntry = driveTable.getEntry("Drive Turn Clockwise");
    NetworkTableEntry driveToPoseEntry = driveTable.getEntry("Pathfind to Pose");
    NetworkTableEntry driveToOtherSideEntry = driveTable.getEntry("Auto Align to Closest Pole");
    NetworkTableEntry resetPoseToVisionEntry = driveTable.getEntry("Reset Pose To Vision");

    driveFeedforwardEntry.setBoolean(false);
    driveSlipCurrentEntry.setBoolean(false);
    driveWheelRadiusEntry.setBoolean(false);
    driveStopXEntry.setBoolean(false);
    driveForwardEntry.setBoolean(false);
    driveClockwiseEntry.setBoolean(false);
    driveToPoseEntry.setBoolean(false);
    driveToOtherSideEntry.setBoolean(false);
    resetPoseToVisionEntry.setBoolean(false);

    Trigger driveFeedforwardTrigger = new Trigger(() -> driveFeedforwardEntry.getBoolean(false));
    Trigger driveSlipCurrentTrigger = new Trigger(() -> driveSlipCurrentEntry.getBoolean(false));
    Trigger driveWheelRadiusTrigger = new Trigger(() -> driveWheelRadiusEntry.getBoolean(false));
    Trigger driveStopXTrigger = new Trigger(() -> driveStopXEntry.getBoolean(false));
    Trigger driveForwardTrigger = new Trigger(() -> driveForwardEntry.getBoolean(false));
    Trigger driveClockwiseTrigger = new Trigger(() -> driveClockwiseEntry.getBoolean(false));
    Trigger driveToPoseTrigger = new Trigger(() -> driveToPoseEntry.getBoolean(false));
    Trigger driveToOtherSideTrigger = new Trigger(() -> driveToOtherSideEntry.getBoolean(false));
    Trigger resetPoseToVisionTrigger = new Trigger(() -> resetPoseToVisionEntry.getBoolean(false));

    // driveFeedforwardTrigger.whileTrue(DriveCommands.feedforwardCharacterization(drive));
    // driveSlipCurrentTrigger.whileTrue(
    //     Commands.print("running slip current test")
    //         .andThen(DriveCommands.slipCurrentCharacterization(drive)));
    // driveWheelRadiusTrigger.whileTrue(DriveCommands.wheelRadiusCharacterization(drive));
    // driveStopXTrigger.onTrue(
    //     Commands.runOnce(drive::stopWithX, drive).andThen(() ->
    // driveStopXEntry.setBoolean(false)));
    // driveForwardTrigger.whileTrue(
    //     Commands.run(() -> drive.runVelocity(new ChassisSpeeds(1, 0.0, 0.0))));
    // driveClockwiseTrigger.whileTrue(
    //     Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 1))));

    driveToPoseTrigger.whileTrue(
        new PathfindToPoseCommand(
            drive,
            () ->
                PoseUtils.getPerpendicularOffsetPose(
                    FieldConstants.redReefCD.rightPole.getPose(), 0.65)));

    // FieldUtils.getClosestReefPole().getPose(), 0.65)));

    driveToOtherSideTrigger.whileTrue(
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

    resetPoseToVisionTrigger.onTrue(
        Commands.runOnce(
            () -> {
              drive.resetOdometry(RobotState.getVisionPose());
            },
            drive));
  }

  private void buildClimberTab() {
    NetworkTable climberTable = NetworkTableInstance.getDefault().getTable("Elastic/Climber");

    NetworkTableEntry climberOutEntry = climberTable.getEntry("Climber Out (While Held)");
    NetworkTableEntry climberDeployEntry = climberTable.getEntry("Climber Deploy (When Pressed)");
    NetworkTableEntry climberClimbEntry = climberTable.getEntry("Climber Climb (When Pressed)");

    climberOutEntry.setBoolean(false);
    climberDeployEntry.setBoolean(false);
    climberClimbEntry.setBoolean(false);

    Trigger climberOutTrigger = new Trigger(() -> climberOutEntry.getBoolean(false));
    Trigger climberDeployTrigger = new Trigger(() -> climberDeployEntry.getBoolean(false));
    Trigger climberClimbTrigger = new Trigger(() -> climberClimbEntry.getBoolean(false));

    climberOutTrigger.whileTrue(climber.climbVoltOut());
    climberOutTrigger.onFalse(climber.climbSTOP());
    climberDeployTrigger.onTrue(
        climber.climbDeploy().andThen(() -> climberDeployEntry.setBoolean(false)));
    climberClimbTrigger.onTrue(
        climber.climbClimb().andThen(() -> climberClimbEntry.setBoolean(false)));
  }

  private void buildTestTab() {
    NetworkTable testTable = NetworkTableInstance.getDefault().getTable("Elastic/Test");

    NetworkTableEntry cleaningEntry = testTable.getEntry("Cleaning Mode");

    cleaningEntry.setBoolean(false);

    Trigger cleaningTrigger = new Trigger(() -> cleaningEntry.getBoolean(false));

    cleaningTrigger.onTrue(new CleaningTest(intake, claw, feeder));
  }

  /** Use this method to define your button->command mappings. */
  private void configureXboxBindings() {

    // Lock to 0° when button is held
    controller
        .b()
        .whileTrue(
            DriveCommands.driveAtAngle(
                drive,
                () ->
                    -controller.getLeftY()
                        * Math.abs(controller.getLeftY())
                        * Constants.DriveConstants.kDriveMaxSpeed,
                () ->
                    -controller.getLeftX()
                        * Math.abs(controller.getLeftX())
                        * Constants.DriveConstants.kDriveMaxSpeed,
                () -> FieldUtils.isRedAlliance() ? Rotation2d.kCCW_90deg : Rotation2d.kCW_90deg));

    // // Auto Align
    controller
        .a()
        .whileTrue(
            new MagicDriveToPoseCommand(
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

    // controller
    //     .rightTrigger();
    //     // .onTrue(
    //         // DriveCommands.
    //     // );

    // Ground algae intake
    controller
        .rightBumper()
        .onTrue(
            Commands.parallel(
                superstructure.setStateCommand(
                    SuperstructureState.INTAKE_ALGAE_GROUND, "GROUND ALGAE"),
                claw.setClawStateCommand(ClawState.ALGAE)));

    // Processor Aim thingy
    controller
        .leftBumper()
        // .onTrue(superstructure.setStateCommand(SuperstructureState.PROCESSOR_AIM, "Aim
        // Processor"));
        .whileTrue(intake.setIntakeStateCommand(IntakeState.REJECT_CORAL));

    // Superstructure Stow
    controller
        .povLeft()
        .onTrue(
            Commands.either(
                superstructure.setStateCommand(SuperstructureState.STOW_ALGAE, "Stow Algae"),
                superstructure.setStateCommand(SuperstructureState.STOW, "Stow"),
                () -> RobotState.hasAlgae()));

    // Intake Stow
    controller
        .povRight()
        .onTrue(
            intake
                .setIntakeStateCommand(IntakeState.STOW)
                .asProxy()
                .alongWith(claw.setClawStateCommand(ClawState.IDLE).asProxy()));

    // Intake ground coral
    controller
        .leftTrigger(0.2)
        .onTrue(
            intake
                .setIntakeStateCommand(IntakeState.INTAKE)
                .asProxy()
                .alongWith(claw.setClawStateCommand(ClawState.INTAKING_CORAL).asProxy()));

    // ALGAE DESCORE PREP
    controller
        .x()
        .onTrue(
            superstructure
                .setStateCommand(
                    () -> robotState.getAlgaeDescoreSuperstructureState(), "Algae Descore Aim")
                .asProxy()
                .alongWith(claw.setClawStateCommand(ClawState.ALGAE))
                .asProxy());

    // Score position Aim
    controller
        .y()
        .onTrue(
            superstructure
                .setStateCommand(() -> robotState.getSuperstructureScoreAimState(), "Aim Scoring")
                .asProxy());

    // controller.back().onTrue(intake.setIntakeStateCommand(IntakeState.SCORING_PREP).asProxy());
    // controller.start().onTrue(intake.setIntakeStateCommand(IntakeState.SCORING).asProxy());
    // controller.povUp().onTrue(intake.setIntakeStateCommand(IntakeState.INTAKE_L1).asProxy());
    // Manual spit out game piece
    controller
        .rightTrigger(0.2) // check
        .onTrue(
            // Commands.either(
            //         intake.setIntakeStateCommand(IntakeState.SCORING).asProxy(),
            Commands.sequence(
                // new PathfindToPoseCommand(
                //     drive,
                //     () ->
                //         PoseUtils.getPerpendicularOffsetPose(
                //             FieldUtils.getClosestReefPole().getPose(), 0.7)),
                // new WaitCommand(0.2)
                new ConditionalCommand(
                    claw.setClawStateCommand(ClawState.SCORING_L1).asProxy(),
                    claw.setClawStateCommand(ClawState.SCORING).asProxy(),
                    () -> robotState.isL1Mode()),
                new WaitUntilCommand(
                        () -> CoralStateTracker.getCurrentPosition() == CoralPosition.NONE)
                    .withTimeout(3),
                superstructure
                    .setStateCommand(() -> robotState.getFadeawayState(), "Aim fade")
                    .asProxy(),
                claw.setClawStateCommand(ClawState.IDLE).asProxy(),
                new ConditionalCommand(
                        new WaitUntilCommand(() -> RobotState.isSafeToStow())
                            .andThen(
                                superstructure
                                    .setStateCommand(SuperstructureState.STOW, "STOW")
                                    .asProxy()),
                        Commands.none(),
                        () -> RobotState.getSuperstructureState().isCoralState())
                    .asProxy())
            // () -> robotState.isL1Mode())
            // .asProxy()
            );

    // controller
    //     .rightTrigger(0.2) // check
    //     .onTrue(
    //         Commands.either(
    //             Commands.sequence(
    //                 new ConditionalCommand(
    //                     claw.setClawStateCommand(ClawState.SCORING_L1).asProxy(),
    //                     claw.setClawStateCommand(ClawState.SCORING).asProxy(),
    //                     () -> robotState.isL1Mode()),
    //                 new WaitUntilCommand(
    //                         () -> CoralStateTracker.getCurrentPosition() == CoralPosition.NONE)
    //                     .withTimeout(3),
    //                 superstructure
    //                     .setStateCommand(() -> robotState.getFadeawayState(), "Aim fade")
    //                     .asProxy(),
    //                 claw.setClawStateCommand(ClawState.IDLE).asProxy(),

    //                 new WaitUntilCommand(() -> RobotState.isSafeToStow()),
    //                 superstructure
    //                     .setStateCommand(SuperstructureState.STOW, "STOW")
    //                     .asProxy()
    //             ),
    //             Commands.sequence(
    //                 new ConditionalCommand(
    //                     claw.setClawStateCommand(ClawState.SCORING_L1).asProxy(),
    //                     claw.setClawStateCommand(ClawState.SCORING).asProxy(),
    //                     () -> robotState.isL1Mode()),
    //                 new WaitUntilCommand(
    //                         () -> CoralStateTracker.getCurrentPosition() == CoralPosition.NONE)
    //                     .withTimeout(3),
    //                 superstructure
    //                     .setStateCommand(() -> robotState.getFadeawayState(), "Aim fade")
    //                     .asProxy(),
    //                 claw.setClawStateCommand(ClawState.IDLE).asProxy()
    //             ),
    //             () -> RobotState.getSuperstructureState().isCoralState()
    //         )
    //     );
  }

  private void configureDriveStreamDeckBindings() {
    StreamDeckButtonConfig orangeConfig =
        new StreamDeckButtonConfig(LedState.kCOOrange.toString(), LedState.kOff.toString(), "");
    StreamDeckButtonConfig tealConfig =
        new StreamDeckButtonConfig(LedState.kCOTeal.toString(), LedState.kWhite.toString(), "");
    StreamDeckButtonConfig tealOnWhiteConfig =
        new StreamDeckButtonConfig(LedState.kWhite.toString(), LedState.kCOTeal.toString(), "");
    StreamDeckButtonConfig orangeOnWhiteConfig =
        new StreamDeckButtonConfig(LedState.kWhite.toString(), LedState.kCOOrange.toString(), "");
    StreamDeckButtonConfig redConfig =
        new StreamDeckButtonConfig(LedState.kRed.toString(), LedState.kOff.toString(), "");
    StreamDeckButtonConfig yellowConfig =
        new StreamDeckButtonConfig(LedState.kYellow.toString(), LedState.kOff.toString(), "");
    StreamDeckButtonConfig yellowOnBlackConfig =
        new StreamDeckButtonConfig(LedState.kOff.toString(), LedState.kYellow.toString(), "");
    StreamDeckButtonConfig activeConfig =
        new StreamDeckButtonConfig(LedState.kGreen.toString(), LedState.kOff.toString(), "");

    StreamDeckButton coralL4Button =
        new StreamDeckButton(0, 7, "Coral L4")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L4");
    StreamDeckButton coralL3Button =
        new StreamDeckButton(1, 7, "Coral L3")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L3");
    StreamDeckButton coralL2Button =
        new StreamDeckButton(2, 7, "Coral L2")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L2");
    StreamDeckButton coralL1Button =
        new StreamDeckButton(3, 7, "Coral L1")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L1");
    StreamDeckButton AlgaeBargeButton =
        new StreamDeckButton(0, 6, "Algae Barge")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("B");
    StreamDeckButton AlgaeL2Button =
        new StreamDeckButton(1, 6, "Algae L2")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("L2");
    StreamDeckButton AlgaeL1Button =
        new StreamDeckButton(2, 6, "Algae L1")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("L1");
    // the Processor is handled by ronny, seperate button
    StreamDeckButton AlgaeProcessorButton =
        new StreamDeckButton(3, 6, "Algae Processor")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("P");
    StreamDeckButton ReefASideButton =
        new StreamDeckButton(3, 1, "Reef A Side")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("A");
    StreamDeckButton ReefBSideButton =
        new StreamDeckButton(2, 2, "Reef B Side")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("B");
    StreamDeckButton ReefCSideButton =
        new StreamDeckButton(1, 2, "Reef C Side")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("C");
    StreamDeckButton ReefDSideButton =
        new StreamDeckButton(0, 1, "Reef D Side")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("D");
    StreamDeckButton ReefESideButton =
        new StreamDeckButton(1, 0, "Reef E Side")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("E");
    StreamDeckButton ReefFSideButton =
        new StreamDeckButton(2, 0, "Reef F Side")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("F");
    StreamDeckButton reefRightSideButton =
        new StreamDeckButton(3, 4, "Reef Right Side 1")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("R");
    StreamDeckButton reefRightSideButton2 =
        new StreamDeckButton(3, 5, "Reef Right Side 2")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("R");
    StreamDeckButton reefLeftSideButton =
        new StreamDeckButton(3, 2, "Reef Left Side 1")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L");
    StreamDeckButton reefLeftSideButton2 =
        new StreamDeckButton(3, 3, "Reef Left Side 2")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L");
    StreamDeckButton homeElevatorButton =
        new StreamDeckButton(2, 4, "Home Elevator")
            .withInactiveConfig(orangeOnWhiteConfig)
            .withActiveConfig(activeConfig)
            .withText("HE");
    StreamDeckButton zeroGyroButton =
        new StreamDeckButton(2, 3, "Zero Gyro 1")
            .withInactiveConfig(tealOnWhiteConfig)
            .withActiveConfig(activeConfig)
            .withText("GZ");
    StreamDeckButton zeroGyroButton2 =
        new StreamDeckButton(2, 5, "Zero Gyro 2")
            .withInactiveConfig(tealOnWhiteConfig)
            .withActiveConfig(activeConfig)
            .withText("GZ");
    StreamDeckButton climbDeployButton =
        new StreamDeckButton(1, 3, "Climb Deploy 1")
            .withInactiveConfig(yellowConfig)
            .withActiveConfig(activeConfig)
            .withText("C");
    StreamDeckButton climbDeployButton2 =
        new StreamDeckButton(1, 5, "Climb Deploy 2")
            .withInactiveConfig(yellowConfig)
            .withActiveConfig(activeConfig)
            .withText("C");
    StreamDeckButton climbClimbButton =
        new StreamDeckButton(0, 3, "Auto Climb 1")
            .withInactiveConfig(redConfig)
            .withActiveConfig(activeConfig)
            .withText("C");
    StreamDeckButton climbClimbButton2 =
        new StreamDeckButton(0, 5, "Auto Climb 2")
            .withInactiveConfig(redConfig)
            .withActiveConfig(activeConfig)
            .withText("C");
    StreamDeckButton manualClimbButton =
        new StreamDeckButton(0, 4, "Manual Climb")
            .withInactiveConfig(yellowOnBlackConfig)
            .withActiveConfig(activeConfig)
            .withText("C");
    StreamDeckButton setManualScoringButton =
        new StreamDeckButton(0, 0, "Manual Score")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("MS");
    StreamDeckButton setAutoScoringButton =
        new StreamDeckButton(0, 2, "Auto Score")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("AS");

    Command homeElevatorButtonCommand = elevator.homeElevator().withName("homeElevatorButton");
    Command climbDelpoyButtonCommand = climber.climbDeploy().withName("climbDeployButton");
    Command climbClimbButtonCommand = climber.climbClimb().withName("climbClimbButton");
    Command manualClimbButtonCommand = climber.climbVoltOut().withName("manualClimbButton");
    Command manualClimbOffButtonCommand = climber.climbSTOP().withName("manualClimbButtonOff");

    Map<StreamDeckButton, BooleanSupplier> customStreamDeckButtonMap = new HashMap<>();

    customStreamDeckButtonMap.put(
        coralL4Button, () -> robotState.getStoredScorePosition().getScoreLevel() == ScoreLevel.L4);
    customStreamDeckButtonMap.put(
        coralL3Button, () -> robotState.getStoredScorePosition().getScoreLevel() == ScoreLevel.L3);
    customStreamDeckButtonMap.put(
        coralL2Button, () -> robotState.getStoredScorePosition().getScoreLevel() == ScoreLevel.L2);
    customStreamDeckButtonMap.put(
        coralL1Button, () -> robotState.getStoredScorePosition().getScoreLevel() == ScoreLevel.L1);
    customStreamDeckButtonMap.put(
        AlgaeBargeButton,
        () -> robotState.getStoredScorePosition().getScoreLevel() == ScoreLevel.BARGE);
    customStreamDeckButtonMap.put(
        AlgaeL2Button,
        () -> robotState.getStoredScorePosition().getAlgaeIntake() == AlgaeIntake.L2_ALGAE);
    customStreamDeckButtonMap.put(
        AlgaeL1Button,
        () -> robotState.getStoredScorePosition().getAlgaeIntake() == AlgaeIntake.L1_ALGAE);
    customStreamDeckButtonMap.put(
        AlgaeProcessorButton,
        () -> robotState.getStoredScorePosition().getScoreLevel() == ScoreLevel.PROCESSOR);
    customStreamDeckButtonMap.put(
        ReefASideButton, () -> robotState.getStoredScorePosition().getReefSide() == ReefSide.A);
    customStreamDeckButtonMap.put(
        ReefBSideButton, () -> robotState.getStoredScorePosition().getReefSide() == ReefSide.B);
    customStreamDeckButtonMap.put(
        ReefCSideButton, () -> robotState.getStoredScorePosition().getReefSide() == ReefSide.C);
    customStreamDeckButtonMap.put(
        ReefDSideButton, () -> robotState.getStoredScorePosition().getReefSide() == ReefSide.D);
    customStreamDeckButtonMap.put(
        ReefESideButton, () -> robotState.getStoredScorePosition().getReefSide() == ReefSide.E);
    customStreamDeckButtonMap.put(
        ReefFSideButton, () -> robotState.getStoredScorePosition().getReefSide() == ReefSide.E);
    customStreamDeckButtonMap.put(
        reefRightSideButton,
        () -> robotState.getStoredScorePosition().getCoralBranch() == CoralBranch.LEFT);
    customStreamDeckButtonMap.put(
        reefRightSideButton2,
        () -> robotState.getStoredScorePosition().getCoralBranch() == CoralBranch.LEFT);
    customStreamDeckButtonMap.put(
        reefLeftSideButton,
        () -> robotState.getStoredScorePosition().getCoralBranch() == CoralBranch.LEFT);
    customStreamDeckButtonMap.put(
        reefLeftSideButton2,
        () -> robotState.getStoredScorePosition().getCoralBranch() == CoralBranch.LEFT);
    customStreamDeckButtonMap.put(homeElevatorButton, homeElevatorButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(climbDeployButton, climbDelpoyButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(climbDeployButton2, climbDelpoyButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(climbClimbButton, climbClimbButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(climbClimbButton2, climbClimbButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(manualClimbButton, manualClimbButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(setManualScoringButton, () -> false);

    streamdeck.configureCustomButtons(customStreamDeckButtonMap);

    streamdeck.configureDefaultButtons(Set.of(zeroGyroButton, zeroGyroButton2));

    streamdeck
        .button(coralL4Button)
        .onTrue(
            Commands.runOnce(() -> robotState.getStoredScorePosition().setScoreLevel(ScoreLevel.L4))
                .ignoringDisable(true));
    streamdeck
        .button(coralL3Button)
        .onTrue(
            Commands.runOnce(() -> robotState.getStoredScorePosition().setScoreLevel(ScoreLevel.L3))
                .ignoringDisable(true));
    streamdeck
        .button(coralL2Button)
        .onTrue(
            Commands.runOnce(() -> robotState.getStoredScorePosition().setScoreLevel(ScoreLevel.L2))
                .ignoringDisable(true));
    streamdeck
        .button(coralL1Button)
        .onTrue(
            Commands.runOnce(() -> robotState.getStoredScorePosition().setScoreLevel(ScoreLevel.L1))
                .ignoringDisable(true));
    streamdeck
        .button(AlgaeBargeButton)
        .onTrue(
            Commands.runOnce(
                    () -> robotState.getStoredScorePosition().setScoreLevel(ScoreLevel.BARGE))
                .ignoringDisable(true));
    streamdeck
        .button(AlgaeL2Button)
        .onTrue(
            Commands.runOnce(
                    () -> {
                      ScorePosition stored = robotState.getStoredScorePosition();
                      if (stored.getAlgaeIntake() == AlgaeIntake.L2_ALGAE) {
                        stored.setAlgaeIntake(AlgaeIntake.NONE);
                      } else {
                        stored.setAlgaeIntake(AlgaeIntake.L2_ALGAE);
                      }
                    })
                .ignoringDisable(true));
    streamdeck
        .button(AlgaeL1Button)
        .onTrue(
            Commands.runOnce(
                    () -> {
                      ScorePosition stored = robotState.getStoredScorePosition();
                      if (stored.getAlgaeIntake() == AlgaeIntake.L1_ALGAE) {
                        stored.setAlgaeIntake(AlgaeIntake.NONE);
                      } else {
                        stored.setAlgaeIntake(AlgaeIntake.L1_ALGAE);
                      }
                    })
                .ignoringDisable(true));

    streamdeck
        .button(AlgaeProcessorButton)
        .onTrue(
            Commands.runOnce(
                    () -> robotState.getStoredScorePosition().setScoreLevel(ScoreLevel.PROCESSOR))
                .ignoringDisable(true));
    streamdeck
        .button(ReefASideButton)
        .onTrue(
            Commands.runOnce(() -> robotState.getStoredScorePosition().setReefSide(ReefSide.A))
                .ignoringDisable(true));
    streamdeck
        .button(ReefBSideButton)
        .onTrue(
            Commands.runOnce(() -> robotState.getStoredScorePosition().setReefSide(ReefSide.B))
                .ignoringDisable(true));
    streamdeck
        .button(ReefCSideButton)
        .onTrue(
            Commands.runOnce(() -> robotState.getStoredScorePosition().setReefSide(ReefSide.C))
                .ignoringDisable(true));
    streamdeck
        .button(ReefDSideButton)
        .onTrue(
            Commands.runOnce(() -> robotState.getStoredScorePosition().setReefSide(ReefSide.D))
                .ignoringDisable(true));
    streamdeck
        .button(ReefESideButton)
        .onTrue(
            Commands.runOnce(() -> robotState.getStoredScorePosition().setReefSide(ReefSide.E))
                .ignoringDisable(true));
    streamdeck
        .button(ReefFSideButton)
        .onTrue(
            Commands.runOnce(() -> robotState.getStoredScorePosition().setReefSide(ReefSide.F))
                .ignoringDisable(true));
    streamdeck
        .button(reefRightSideButton)
        .onTrue(
            Commands.runOnce(
                    () -> robotState.getStoredScorePosition().setBranchSide(CoralBranch.RIGHT))
                .ignoringDisable(true));
    streamdeck
        .button(reefRightSideButton2)
        .onTrue(
            Commands.runOnce(
                    () -> robotState.getStoredScorePosition().setBranchSide(CoralBranch.RIGHT))
                .ignoringDisable(true));
    streamdeck
        .button(reefLeftSideButton)
        .onTrue(
            Commands.runOnce(
                    () -> robotState.getStoredScorePosition().setBranchSide(CoralBranch.LEFT))
                .ignoringDisable(true));
    streamdeck
        .button(reefLeftSideButton2)
        .onTrue(
            Commands.runOnce(
                    () -> robotState.getStoredScorePosition().setBranchSide(CoralBranch.LEFT))
                .ignoringDisable(true));
    streamdeck.button(homeElevatorButton).onTrue(homeElevatorButtonCommand);
    streamdeck
        .button(setManualScoringButton)
        .onTrue(Commands.runOnce(() -> robotState.setScoringModeManual()));
    streamdeck
        .button(setAutoScoringButton)
        .onTrue(Commands.runOnce(() -> robotState.setScoringModeAuto()));

    streamdeck
        .button(climbDeployButton)
        .and(streamdeck.button(climbDeployButton2))
        .onTrue(
            Commands.parallel(
                climbDelpoyButtonCommand,
                superstructure.setStateCommand(SuperstructureState.CLIMB, "Climb")));
    streamdeck
        .button(climbClimbButton)
        .and(streamdeck.button(climbClimbButton2))
        .and(streamdeck.button(manualClimbButton).negate())
        .onTrue(climbClimbButtonCommand);
    streamdeck
        .button(manualClimbButton)
        .and(streamdeck.button(climbClimbButton))
        .and(streamdeck.button(climbClimbButton2))
        .onTrue(manualClimbButtonCommand);
    streamdeck.button(climbDeployButton).onFalse(manualClimbOffButtonCommand);
    streamdeck.button(climbDeployButton2).onFalse(manualClimbOffButtonCommand);
    streamdeck.button(climbClimbButton).onFalse(manualClimbOffButtonCommand);
    streamdeck.button(climbClimbButton2).onFalse(manualClimbOffButtonCommand);
    streamdeck.button(manualClimbButton).onFalse(manualClimbOffButtonCommand);
    streamdeck
        .button(zeroGyroButton)
        .and(streamdeck.button(zeroGyroButton2))
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.resetOdometry(
                            new Pose2d(
                                RobotState.getGlobalPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // manualClimbOffButtonCommand

  }

  private void configureTestingStreamDeckBindings() {
    StreamDeckButtonConfig orangeConfig =
        new StreamDeckButtonConfig(LedState.kCOOrange.toString(), LedState.kOff.toString(), "");
    StreamDeckButtonConfig tealConfig =
        new StreamDeckButtonConfig(LedState.kCOTeal.toString(), LedState.kWhite.toString(), "");
    StreamDeckButtonConfig grayConfig =
        new StreamDeckButtonConfig(LedState.kGray.toString(), LedState.kWhite.toString(), "");
    StreamDeckButtonConfig redConfig =
        new StreamDeckButtonConfig(LedState.kRed.toString(), LedState.kOff.toString(), "");
    StreamDeckButtonConfig activeConfig =
        new StreamDeckButtonConfig(LedState.kWhite.toString(), LedState.kOff.toString(), "");

    // StreamDeckButton swerveXButton =
    //     new StreamDeckButton(3, 7, "Swerve X")
    //         .withInactiveConfig(inactiveConfig)
    //         .withActiveConfig(activeConfig)
    //         .withInactiveText("X")
    //         .withActiveText("Swerve X");
    StreamDeckButton intakeInButton =
        new StreamDeckButton(1, 0, "Intake In")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("INT In");
    StreamDeckButton intakeOutButton =
        new StreamDeckButton(1, 1, "Intake Out")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("INT Out");
    StreamDeckButton intakeUpButton =
        new StreamDeckButton(0, 0, "Intake Up")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("INT Up");
    StreamDeckButton intakeDownButton =
        new StreamDeckButton(0, 1, "Intake Down")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("INT Down");
    StreamDeckButton feederInButton =
        new StreamDeckButton(2, 0, "Feeder In ")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("FEED In");
    StreamDeckButton feederOutButton =
        new StreamDeckButton(2, 1, "Feeder Out")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("FEED Out ");
    StreamDeckButton intakePosScoreButton =
        new StreamDeckButton(1, 2, "Intake PosScore")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("INT PosScore");
    StreamDeckButton intakeDefaultButton =
        new StreamDeckButton(0, 3, "Intake Default")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("INT Default");
    StreamDeckButton intakeZeroButton =
        new StreamDeckButton(3, 2, "Intake Zero")
            .withInactiveConfig(redConfig)
            .withActiveConfig(activeConfig)
            .withText("INT Zero");
    StreamDeckButton noneButton =
        new StreamDeckButton(1, 3, "none")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("NONE");
    StreamDeckButton stowButton =
        new StreamDeckButton(3, 3, "stow")
            .withInactiveConfig(grayConfig)
            .withActiveConfig(activeConfig)
            .withText("STOW");
    StreamDeckButton stowCoralButton =
        new StreamDeckButton(2, 3, "stowCoral")
            .withInactiveConfig(grayConfig)
            .withActiveConfig(activeConfig)
            .withText("STOW C");
    StreamDeckButton stowAlgaeButton =
        new StreamDeckButton(1, 3, "stowAlgae")
            .withInactiveConfig(grayConfig)
            .withActiveConfig(activeConfig)
            .withText("STOW A");
    // StreamDeckButton intakeCoralButton =
    //     new StreamDeckButton(3,7, "intakeCoral")
    //         .withInactiveConfig(inactiveConfig)
    //         .withActiveConfig(activeConfig)
    //         .withText("intakeCoral");
    // StreamDeckButton intakeCoralL1Button =
    //     new StreamDeckButton(3,7, "intakeCoralL1")
    //         .withInactiveConfig(inactiveConfig)
    //         .withActiveConfig(activeConfig)
    //         .withText("intakeCoralL1");
    // StreamDeckButton feedButton =
    //     new StreamDeckButton(3,7, "feed")
    //         .withInactiveConfig(inactiveConfig)
    //         .withActiveConfig(activeConfig)
    //         .withText("feed");
    StreamDeckButton l1PivotButton =
        new StreamDeckButton(3, 5, "l1Pivot")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L1 PIV");
    StreamDeckButton l2FadeawayButton =
        new StreamDeckButton(2, 5, "l2Fadeaway")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L2 F");
    StreamDeckButton l3FadeawayButton =
        new StreamDeckButton(1, 5, "l3Fadeaway")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L3 F");
    StreamDeckButton l4FadeawayButton =
        new StreamDeckButton(0, 5, "l4Fadeaway")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L4 F");
    StreamDeckButton l1ScoreButton =
        new StreamDeckButton(3, 4, "l1Score")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L1 S");
    StreamDeckButton l2ScoreButton =
        new StreamDeckButton(2, 4, "l2Score")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L2 S");
    StreamDeckButton l3ScoreButton =
        new StreamDeckButton(1, 4, "l3Score")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L3 S");
    StreamDeckButton l4ScoreButton =
        new StreamDeckButton(0, 4, "l4Score")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L4 S");
    StreamDeckButton algaeHighIntakeButton =
        new StreamDeckButton(1, 6, "algaeHighIntake")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("ALG H");
    StreamDeckButton algaeLowIntakeButton =
        new StreamDeckButton(2, 6, "algaeLowIntake")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("ALG L");
    StreamDeckButton processorAimButton =
        new StreamDeckButton(3, 7, "processorAim")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("P");
    StreamDeckButton bargeAimCenterButton =
        new StreamDeckButton(1, 7, "bargeAimCenter")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("B-C");
    StreamDeckButton bargeAimForwardButton =
        new StreamDeckButton(2, 7, "bargeAimForward")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("B-F");
    StreamDeckButton bargeAimBackwardButton =
        new StreamDeckButton(0, 7, "bargeAimBackward")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("B-B");

    // Command swerveXButtonCommand =
    // Commands.runOnce(drive::stopWithX, drive).withName("swerveXButton");
    Command intakeInButtonCommand = intake.intakeFWD().withName("intakeInButton");
    Command intakeInButtonOffCommand = intake.intakeSTOP().withName("intakeInButtonOff");
    Command intakeOutButtonCommand = intake.intakeRVS().withName("intakeOutButton");
    Command intakeOutButtonOffCommand = intake.intakeSTOP().withName("intakeOutButtonOff");
    Command intakeUpButtonCommand = intake.pivotManualTestForward().withName("intakeUpButton");
    Command intakeUpButtonOffCommand = intake.pivotStop().withName("intakeUpButtonOff");
    Command intakeDownButtonCommand = intake.pivotManualTestReverse().withName("intakeDownButton");
    Command intakeDownButtonOffCommand = intake.pivotStop().withName("intakeDownButtonOff");
    // Command intakeL1UpButtonCommand =
    //     intake.().withName("intakeL1UpButton");
    Command feederInButtonCommand = intake.feederFWD().withName("feederInButton");
    Command feederInButtonOffCommand = intake.feederSTOP().withName("feederInButtonOff");
    Command feederOutButtonCommand = intake.feederRVS().withName("feederOutButton");
    Command feederOutButtonOffCommand = intake.feederSTOP().withName("feederOutButtonOff");
    // Command intakePosUpButtonCommand = intake.setPivotUp().withName("intakePosUpButton");
    // Command intakePosDownButtonCommand = intake.setPivotDown().withName("intakePosDownButton");
    Command intakePosScoreButtonCommand = intake.setPivotScoring().withName("intakePosScoreButton");
    Command intakeDefaultButtonCommand =
        Commands.runOnce(() -> CommandScheduler.getInstance().cancel(intake.getCurrentCommand()))
            .withName("intakeDefaultButton");
    Command intakeZeroButtonCommand = intake.zeroPivotAtPivotUp().withName("intakeZeroButton");

    // streamdeck.configureDefaultButtons(
    //     Set.of(
    //         swerveXButton,
    //         intakeInButton,
    //         intakeOutButton,
    //         intakeUpButton,
    //         intakeDownButton,
    //         intakeL1UpButton,
    //         intakeL1DownButton,
    //         feederInButton,
    //         feederOutButton,
    //         intakeZeroButton));

    // streamdeck.configureToggleButtons(
    //     Set.of(intakePosUpButton, intakePosDownButton, intakePosScoreButton));

    Map<StreamDeckButton, BooleanSupplier> customStreamDeckButtonMap = new HashMap<>();

    // customStreamDeckButtonMap.put(swerveXButton, swerveXButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(intakeInButton, intakeInButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(intakeOutButton, intakeOutButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(intakeUpButton, intakeUpButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(intakeDownButton, intakeDownButtonCommand::isScheduled);
    // customStreamDeckButtonMap.put(intakeL1UpButton, intakeL1UpButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(feederInButton, feederInButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(feederOutButton, feederOutButtonCommand::isScheduled);
    // customStreamDeckButtonMap.put(intakePosUpButton, intakePosUpButtonCommand::isScheduled);
    // customStreamDeckButtonMap.put(intakePosDownButton, intakePosDownButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(intakePosScoreButton, intakePosScoreButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(intakeDefaultButton, intake.getDefaultCommand()::isScheduled);
    customStreamDeckButtonMap.put(intakeZeroButton, intakeZeroButtonCommand::isScheduled);

    customStreamDeckButtonMap.put(
        noneButton, () -> superstructure.getCurrentState() == SuperstructureState.NONE);
    customStreamDeckButtonMap.put(
        stowButton, () -> superstructure.getCurrentState() == SuperstructureState.STOW);
    customStreamDeckButtonMap.put(
        stowCoralButton, () -> superstructure.getCurrentState() == SuperstructureState.STOW_CORAL);
    customStreamDeckButtonMap.put(
        stowAlgaeButton, () -> superstructure.getCurrentState() == SuperstructureState.STOW_ALGAE);
    // customStreamDeckButtonMap.put(intakeCoralButton, () -> superstructure.getCurrentState() ==
    // SuperstructureState.INTAKE_CORAL);
    // customStreamDeckButtonMap.put(intakeCoralL1Button, () -> superstructure.getCurrentState() ==
    // SuperstructureState.INTAKE_CORAL_L1);
    // customStreamDeckButtonMap.put(feedButton, () -> superstructure.getCurrentState() ==
    // SuperstructureState.FEED);
    customStreamDeckButtonMap.put(
        l1PivotButton, () -> superstructure.getCurrentState() == SuperstructureState.L1_PIVOT);
    customStreamDeckButtonMap.put(
        l2FadeawayButton,
        () -> superstructure.getCurrentState() == SuperstructureState.L2_FADEAWAY);
    customStreamDeckButtonMap.put(
        l3FadeawayButton,
        () -> superstructure.getCurrentState() == SuperstructureState.L3_FADEAWAY);
    customStreamDeckButtonMap.put(
        l4FadeawayButton,
        () -> superstructure.getCurrentState() == SuperstructureState.L4_FADEAWAY);
    customStreamDeckButtonMap.put(
        l1ScoreButton, () -> superstructure.getCurrentState() == SuperstructureState.L1_PIVOT);
    customStreamDeckButtonMap.put(
        l2ScoreButton, () -> superstructure.getCurrentState() == SuperstructureState.L2_AIM);
    customStreamDeckButtonMap.put(
        l3ScoreButton, () -> superstructure.getCurrentState() == SuperstructureState.L3_AIM);
    customStreamDeckButtonMap.put(
        l4ScoreButton, () -> superstructure.getCurrentState() == SuperstructureState.L4_AIM);
    customStreamDeckButtonMap.put(
        algaeHighIntakeButton,
        () -> superstructure.getCurrentState() == SuperstructureState.ALGAE_HIGH_INTAKE);
    customStreamDeckButtonMap.put(
        algaeLowIntakeButton,
        () -> superstructure.getCurrentState() == SuperstructureState.ALGAE_LOW_INTAKE);
    customStreamDeckButtonMap.put(
        processorAimButton,
        () -> superstructure.getCurrentState() == SuperstructureState.PROCESSOR_AIM);
    customStreamDeckButtonMap.put(
        bargeAimCenterButton,
        () -> superstructure.getCurrentState() == SuperstructureState.BARGE_AIM_CENTER);
    customStreamDeckButtonMap.put(
        bargeAimForwardButton,
        () -> superstructure.getCurrentState() == SuperstructureState.BARGE_AIM_FORWARD);
    customStreamDeckButtonMap.put(
        bargeAimBackwardButton,
        () -> superstructure.getCurrentState() == SuperstructureState.BARGE_AIM_BACKWARD);

    streamdeck.configureCustomButtons(customStreamDeckButtonMap);

    // streamdeck.button(swerveXButton).onTrue(swerveXButtonCommand);
    streamdeck.button(intakeInButton).whileTrue(intakeInButtonCommand);
    streamdeck.button(intakeInButton).onFalse(intakeInButtonOffCommand);
    streamdeck.button(intakeOutButton).whileTrue(intakeOutButtonCommand);
    streamdeck.button(intakeOutButton).onFalse(intakeOutButtonOffCommand);
    streamdeck.button(intakeUpButton).whileTrue(intakeUpButtonCommand);
    streamdeck.button(intakeUpButton).onFalse(intakeUpButtonOffCommand);
    streamdeck.button(intakeDownButton).whileTrue(intakeDownButtonCommand);
    streamdeck.button(intakeDownButton).onFalse(intakeDownButtonOffCommand);
    // streamdeck.button(intakeL1UpButton).whileTrue(intakeL1UpButtonCommand);
    streamdeck.button(feederInButton).whileTrue(feederInButtonCommand);
    streamdeck.button(feederInButton).onFalse(feederInButtonOffCommand);
    streamdeck.button(feederOutButton).whileTrue(feederOutButtonCommand);
    streamdeck.button(feederOutButton).onFalse(feederOutButtonOffCommand);
    // streamdeck.button(intakePosUpButton).onTrue(intakePosUpButtonCommand);
    // streamdeck.button(intakePosDownButton).onTrue(intakePosDownButtonCommand);
    streamdeck.button(intakePosScoreButton).onTrue(intakePosScoreButtonCommand);
    streamdeck.button(intakeDefaultButton).onTrue(intakeDefaultButtonCommand);
    streamdeck.button(intakeZeroButton).onTrue(intakeZeroButtonCommand);

    streamdeck
        .button(noneButton)
        .onTrue(superstructure.setStateCommand(SuperstructureState.NONE, "Set NONE"));
    streamdeck
        .button(stowButton)
        .onTrue(superstructure.setStateCommand(SuperstructureState.STOW, "Set STOW"));
    streamdeck
        .button(stowCoralButton)
        .onTrue(superstructure.setStateCommand(SuperstructureState.STOW_CORAL, "Set STOW_CORAL"));
    streamdeck
        .button(stowAlgaeButton)
        .onTrue(superstructure.setStateCommand(SuperstructureState.STOW_ALGAE, "Set STOW_ALGAE"));
    // streamdeck.button(intakeCoralButton).onTrue(superstructure.setStateCommand(SuperstructureState.INTAKE_CORAL, "Set INTAKE_CORAL"));
    // streamdeck.button(intakeCoralL1Button).onTrue(superstructure.setStateCommand(SuperstructureState.INTAKE_CORAL_L1, "Set INTAKE_CORAL_L1"));
    // streamdeck.button(feedButton).onTrue(superstructure.setStateCommand(SuperstructureState.FEED,
    // "Set FEED"));
    streamdeck
        .button(l1PivotButton)
        .onTrue(superstructure.setStateCommand(SuperstructureState.L1_PIVOT, "Set L1_PIVOT"));
    streamdeck
        .button(l2FadeawayButton)
        .onTrue(superstructure.setStateCommand(SuperstructureState.L2_FADEAWAY, "Set L2_FADEAWAY"));
    streamdeck
        .button(l3FadeawayButton)
        .onTrue(superstructure.setStateCommand(SuperstructureState.L3_FADEAWAY, "Set L3_FADEAWAY"));
    streamdeck
        .button(l4FadeawayButton)
        .onTrue(superstructure.setStateCommand(SuperstructureState.L4_FADEAWAY, "Set L4_FADEAWAY"));
    streamdeck
        .button(l1ScoreButton)
        .onTrue(superstructure.setStateCommand(SuperstructureState.L1_PIVOT, "Set L1_AIM"));
    streamdeck
        .button(l2ScoreButton)
        .onTrue(superstructure.setStateCommand(SuperstructureState.L2_AIM, "Set L2_AIM"));
    streamdeck
        .button(l3ScoreButton)
        .onTrue(superstructure.setStateCommand(SuperstructureState.L3_AIM, "Set L3_AIM"));
    streamdeck
        .button(l4ScoreButton)
        .onTrue(superstructure.setStateCommand(SuperstructureState.L4_AIM, "Set L4_AIM"));
    streamdeck
        .button(algaeHighIntakeButton)
        .onTrue(
            superstructure.setStateCommand(
                SuperstructureState.ALGAE_HIGH_INTAKE, "Set ALGAE_HIGH_INTAKE"));
    streamdeck
        .button(algaeLowIntakeButton)
        .onTrue(
            superstructure.setStateCommand(
                SuperstructureState.ALGAE_LOW_INTAKE, "Set ALGAE_LOW_INTAKE"));
    streamdeck
        .button(processorAimButton)
        .onTrue(
            superstructure.setStateCommand(SuperstructureState.PROCESSOR_AIM, "Set PROCESSOR_AIM"));
    streamdeck
        .button(bargeAimCenterButton)
        .onTrue(
            superstructure.setStateCommand(
                SuperstructureState.BARGE_AIM_CENTER, "Set BARGE_AIM_CENTER"));
    streamdeck
        .button(bargeAimForwardButton)
        .onTrue(
            superstructure.setStateCommand(
                SuperstructureState.BARGE_AIM_FORWARD, "Set BARGE_AIM_FORWARD"));
    streamdeck
        .button(bargeAimBackwardButton)
        .onTrue(
            superstructure.setStateCommand(
                SuperstructureState.BARGE_AIM_BACKWARD, "Set BARGE_AIM_BACKWARD"));
  }

  private void configureArbitraryTriggers() {
    feeder.dejamTrigger.onTrue(intake.dejamFeeder());
    elevator.elevatorObjectTrigger.onTrue(elevator.dejamElevator());
    intake.rejectCoralTrigger().whileTrue(intake.rejectCoralCommand());

    //
    Trigger autoStowAlgaeTrigger =
        new Trigger(
            () ->
                RobotState.hasAlgae()
                    && RobotState.getSuperstructureState()
                        == SuperstructureState.INTAKE_ALGAE_GROUND);

    autoStowAlgaeTrigger
        .debounce(0.2)
        .onTrue(superstructure.setStateCommand(SuperstructureState.STOW_ALGAE, "Auto Stow Algae"));

    Trigger hasAlgaeHaptics = new Trigger(() -> RobotState.hasAlgae());

    hasAlgaeHaptics.whileTrue(
        Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0.5))
            .andThen(new WaitCommand(0.5))
            .andThen(() -> controller.setRumble(RumbleType.kBothRumble, 0.0)));

    // RobotState.finishedBargeScoringForward()
    //     .onTrue(
    //         superstructure.setStateCommand(
    //             SuperstructureState.BARGE_AIM_CENTER, "Auto set BARGE_AIM_CENTER after
    // scoring"));
    // RobotState.finishedBargeScoringBackward()
    //     .onTrue(
    //         superstructure.setStateCommand(
    //             SuperstructureState.STOW, "Auto set BARGE_AIM_CENTER after scoring"));
  }

  private void configureSuperstructureTrigger() {
    superstructure.setTriggers();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Elevator getElevator() {
    return elevator;
  }

  public EndEffector getEndEffector() {
    return endEffector;
  }

  public Intake getIntake() {
    return intake;
  }
}
