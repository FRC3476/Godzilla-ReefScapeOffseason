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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.test.DrivetrainTest;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.end_effector.EndEffector;
import frc.robot.subsystems.end_effector.EndEffectorIO;
import frc.robot.subsystems.end_effector.EndEffectorIOReal;
import frc.robot.subsystems.end_effector.EndEffectorIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final EndEffector endEffector;
  private final Elevator elevator;
  private final Superstructure superstructure;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        intake = new Intake(new IntakeIOReal());
        endEffector = new EndEffector(new EndEffectorIOReal());
        elevator = new Elevator(new ElevatorIOReal());
        superstructure = new Superstructure(elevator, endEffector);
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight),
                superstructure);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        intake = new Intake(new IntakeIOSim());
        endEffector = new EndEffector(new EndEffectorIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        superstructure = new Superstructure(elevator, endEffector);
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight),
                superstructure);
        break;

      default:
        // Replayed robot, disable IO implementations
        intake = new Intake(new IntakeIO() {});
        endEffector = new EndEffector(new EndEffectorIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        superstructure = new Superstructure(elevator, endEffector);
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                superstructure);
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive Slip Current Characterization (Wall Test)",
        DriveCommands.slipCurrentCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption("Drivetrain Test", new DrivetrainTest(drive));

    BuildIntakeTab();
    BuildEndEffectorTab();
    BuildElevatorTab();
    BuildSuperstructureTab();
    BuildDriveTab();

    RegisterDefaultCommands();

    // Configure the button bindings
    configureButtonBindings();

    // Configure arbitrary triggers
    configureArbitraryTriggers();
  }

  private void BuildIntakeTab() {
    // Get the NetworkTable for the Intake tab
    NetworkTable intakeTable = NetworkTableInstance.getDefault().getTable("Intake");

    // Create NetworkTableEntry instances for while-held functionality
    NetworkTableEntry intakeForwardEntry = intakeTable.getEntry("Intake Forward (While Held)");
    NetworkTableEntry intakeReverseEntry = intakeTable.getEntry("Intake Reverse (While Held)");

    // Initialize entries with default values
    intakeForwardEntry.setBoolean(false);
    intakeReverseEntry.setBoolean(false);

    // Create triggers based on the NetworkTableEntry values
    Trigger intakeForwardTrigger = new Trigger(() -> intakeForwardEntry.getBoolean(false));
    Trigger intakeReverseTrigger = new Trigger(() -> intakeReverseEntry.getBoolean(false));

    // Configure the while-held behavior
    intakeForwardTrigger.whileTrue(intake.intakeFWD());
    intakeForwardTrigger.onFalse(intake.intakeSTOP());

    intakeReverseTrigger.whileTrue(intake.intakeRVS());
    intakeReverseTrigger.onFalse(intake.intakeSTOP());
  }

  private void BuildEndEffectorTab() {
    // Get the NetworkTable for the EndEffector tab
    NetworkTable endEffectorTable = NetworkTableInstance.getDefault().getTable("EndEffector");

    // Create NetworkTableEntry instances for while-held functionality
    NetworkTableEntry endEffectorForwardEntry =
        endEffectorTable.getEntry("Roller Forward (While Held)");
    NetworkTableEntry endEffectorReverseEntry =
        endEffectorTable.getEntry("Roller Reverse (While Held)");

    // Initialize entries with default values
    endEffectorForwardEntry.setBoolean(false);
    endEffectorReverseEntry.setBoolean(false);

    // Create triggers based on the NetworkTableEntry values
    Trigger endEffectorForwardTrigger =
        new Trigger(() -> endEffectorForwardEntry.getBoolean(false));
    Trigger endEffectorReverseTrigger =
        new Trigger(() -> endEffectorReverseEntry.getBoolean(false));
    // Configure the while-held behavior
    endEffectorForwardTrigger.whileTrue(endEffector.rollerFWD());
    endEffectorForwardTrigger.onFalse(endEffector.rollerSTOP());

    endEffectorReverseTrigger.whileTrue(endEffector.rollerRVS());
    endEffectorReverseTrigger.onFalse(endEffector.rollerSTOP());
  }

  private void BuildElevatorTab() {
    // Get the NetworkTable for the Elevator tab
    NetworkTable elevatorTable = NetworkTableInstance.getDefault().getTable("Elevator");

    // Create NetworkTableEntry instances for while-held functionality
    NetworkTableEntry elevatorUpEntry = elevatorTable.getEntry("Elevator Up (While Held)");
    NetworkTableEntry elevatorDownEntry = elevatorTable.getEntry("Elevator Down (While Held)");

    // Initialize entries with default values
    elevatorUpEntry.setBoolean(false);
    elevatorDownEntry.setBoolean(false);

    // Create triggers based on the NetworkTableEntry values
    Trigger elevatorUpTrigger = new Trigger(() -> elevatorUpEntry.getBoolean(false));
    Trigger elevatorDownTrigger = new Trigger(() -> elevatorDownEntry.getBoolean(false));

    // Configure the while-held behavior
    elevatorUpTrigger.whileTrue(elevator.elevatorUP());
    elevatorUpTrigger.onFalse(elevator.elevatorSTOP());

    elevatorDownTrigger.whileTrue(elevator.elevatorDWN());
    elevatorDownTrigger.onFalse(elevator.elevatorSTOP());
  }

  private void BuildSuperstructureTab() {
    // Get the NetworkTable for the Superstructure tab
    NetworkTable superstructureTable = NetworkTableInstance.getDefault().getTable("Superstructure");

    // Create NetworkTableEntry instances for each SuperstructureState
    NetworkTableEntry stowEntry = superstructureTable.getEntry("STOW");
    NetworkTableEntry stowCoralEntry = superstructureTable.getEntry("STOW_CORAL");
    NetworkTableEntry stowAlgaeEntry = superstructureTable.getEntry("STOW_ALGAE");
    NetworkTableEntry intakeCoralEntry = superstructureTable.getEntry("INTAKE_CORAL");
    NetworkTableEntry intakeCoralL1Entry = superstructureTable.getEntry("INTAKE_CORAL_L1");
    NetworkTableEntry feedEntry = superstructureTable.getEntry("FEED");
    NetworkTableEntry l1PivotEntry = superstructureTable.getEntry("L1_PIVOT");
    NetworkTableEntry l2AimEntry = superstructureTable.getEntry("L2_AIM");
    NetworkTableEntry l3AimEntry = superstructureTable.getEntry("L3_AIM");
    NetworkTableEntry l4AimEntry = superstructureTable.getEntry("L4_AIM");
    NetworkTableEntry l1ScoreEntry = superstructureTable.getEntry("L1_SCORE");
    NetworkTableEntry l2ScoreEntry = superstructureTable.getEntry("L2_SCORE");
    NetworkTableEntry l3ScoreEntry = superstructureTable.getEntry("L3_SCORE");
    NetworkTableEntry l4ScoreEntry = superstructureTable.getEntry("L4_SCORE");
    NetworkTableEntry algaeHighIntakeEntry = superstructureTable.getEntry("ALGAE_HIGH_INTAKE");
    NetworkTableEntry algaeLowIntakeEntry = superstructureTable.getEntry("ALGAE_LOW_INTAKE");
    NetworkTableEntry processorAimEntry = superstructureTable.getEntry("PROCESSOR_AIM");
    NetworkTableEntry bargeAimCenterEntry = superstructureTable.getEntry("BARGE_AIM_CENTER");
    NetworkTableEntry bargeAimForwardEntry = superstructureTable.getEntry("BARGE_AIM_FORWARD");
    NetworkTableEntry bargeAimBackwardEntry = superstructureTable.getEntry("BARGE_AIM_BACKWARD");

    // Initialize entries with default values
    stowEntry.setBoolean(false);
    stowCoralEntry.setBoolean(false);
    stowAlgaeEntry.setBoolean(false);
    intakeCoralEntry.setBoolean(false);
    intakeCoralL1Entry.setBoolean(false);
    feedEntry.setBoolean(false);
    l1PivotEntry.setBoolean(false);
    l2AimEntry.setBoolean(false);
    l3AimEntry.setBoolean(false);
    l4AimEntry.setBoolean(false);
    l1ScoreEntry.setBoolean(false);
    l2ScoreEntry.setBoolean(false);
    l3ScoreEntry.setBoolean(false);
    l4ScoreEntry.setBoolean(false);
    algaeHighIntakeEntry.setBoolean(false);
    algaeLowIntakeEntry.setBoolean(false);
    processorAimEntry.setBoolean(false);
    bargeAimCenterEntry.setBoolean(false);
    bargeAimForwardEntry.setBoolean(false);
    bargeAimBackwardEntry.setBoolean(false);

    // Create triggers for each button
    Trigger stowTrigger = new Trigger(() -> stowEntry.getBoolean(false));
    Trigger stowCoralTrigger = new Trigger(() -> stowCoralEntry.getBoolean(false));
    Trigger stowAlgaeTrigger = new Trigger(() -> stowAlgaeEntry.getBoolean(false));
    Trigger intakeCoralTrigger = new Trigger(() -> intakeCoralEntry.getBoolean(false));
    Trigger intakeCoralL1Trigger = new Trigger(() -> intakeCoralL1Entry.getBoolean(false));
    Trigger feedTrigger = new Trigger(() -> feedEntry.getBoolean(false));
    Trigger l1PivotTrigger = new Trigger(() -> l1PivotEntry.getBoolean(false));
    Trigger l2AimTrigger = new Trigger(() -> l2AimEntry.getBoolean(false));
    Trigger l3AimTrigger = new Trigger(() -> l3AimEntry.getBoolean(false));
    Trigger l4AimTrigger = new Trigger(() -> l4AimEntry.getBoolean(false));
    Trigger l1ScoreTrigger = new Trigger(() -> l1ScoreEntry.getBoolean(false));
    Trigger l2ScoreTrigger = new Trigger(() -> l2ScoreEntry.getBoolean(false));
    Trigger l3ScoreTrigger = new Trigger(() -> l3ScoreEntry.getBoolean(false));
    Trigger l4ScoreTrigger = new Trigger(() -> l4ScoreEntry.getBoolean(false));
    Trigger algaeHighIntakeTrigger = new Trigger(() -> algaeHighIntakeEntry.getBoolean(false));
    Trigger algaeLowIntakeTrigger = new Trigger(() -> algaeLowIntakeEntry.getBoolean(false));
    Trigger processorAimTrigger = new Trigger(() -> processorAimEntry.getBoolean(false));
    Trigger bargeAimCenterTrigger = new Trigger(() -> bargeAimCenterEntry.getBoolean(false));
    Trigger bargeAimForwardTrigger = new Trigger(() -> bargeAimForwardEntry.getBoolean(false));
    Trigger bargeAimBackwardTrigger = new Trigger(() -> bargeAimBackwardEntry.getBoolean(false));

    // Wire triggers to superstructure state commands
    stowTrigger.onTrue(superstructure.setStateCommand(SuperstructureState.STOW, "Set STOW"));
    stowCoralTrigger.onTrue(superstructure.setStateCommand(SuperstructureState.STOW_CORAL, "Set STOW_CORAL"));
    stowAlgaeTrigger.onTrue(superstructure.setStateCommand(SuperstructureState.STOW_ALGAE, "Set STOW_ALGAE"));
    intakeCoralTrigger.onTrue(superstructure.setStateCommand(SuperstructureState.INTAKE_CORAL, "Set INTAKE_CORAL"));
    intakeCoralL1Trigger.onTrue(superstructure.setStateCommand(SuperstructureState.INTAKE_CORAL_L1, "Set INTAKE_CORAL_L1"));
    feedTrigger.onTrue(superstructure.setStateCommand(SuperstructureState.FEED, "Set FEED"));
    l1PivotTrigger.onTrue(superstructure.setStateCommand(SuperstructureState.L1_PIVOT, "Set L1_PIVOT"));
    l2AimTrigger.onTrue(superstructure.setStateCommand(SuperstructureState.L2_AIM, "Set L2_AIM"));
    l3AimTrigger.onTrue(superstructure.setStateCommand(SuperstructureState.L3_AIM, "Set L3_AIM"));
    l4AimTrigger.onTrue(superstructure.setStateCommand(SuperstructureState.L4_AIM, "Set L4_AIM"));
    l1ScoreTrigger.onTrue(superstructure.setStateCommand(SuperstructureState.L1_SCORE, "Set L1_SCORE"));
    l2ScoreTrigger.onTrue(superstructure.setStateCommand(SuperstructureState.L2_SCORE, "Set L2_SCORE"));
    l3ScoreTrigger.onTrue(superstructure.setStateCommand(SuperstructureState.L3_SCORE, "Set L3_SCORE"));
    l4ScoreTrigger.onTrue(superstructure.setStateCommand(SuperstructureState.L4_SCORE, "Set L4_SCORE"));
    algaeHighIntakeTrigger.onTrue(superstructure.setStateCommand(SuperstructureState.ALGAE_HIGH_INTAKE, "Set ALGAE_HIGH_INTAKE"));
    algaeLowIntakeTrigger.onTrue(superstructure.setStateCommand(SuperstructureState.ALGAE_LOW_INTAKE, "Set ALGAE_LOW_INTAKE"));
    processorAimTrigger.onTrue(superstructure.setStateCommand(SuperstructureState.PROCESSOR_AIM, "Set PROCESSOR_AIM"));
    bargeAimCenterTrigger.onTrue(superstructure.setStateCommand(SuperstructureState.BARGE_AIM_CENTER, "Set BARGE_AIM_CENTER"));
    bargeAimForwardTrigger.onTrue(superstructure.setStateCommand(SuperstructureState.BARGE_AIM_FORWARD, "Set BARGE_AIM_FORWARD"));
    bargeAimBackwardTrigger.onTrue(superstructure.setStateCommand(SuperstructureState.BARGE_AIM_BACKWARD, "Set BARGE_AIM_BACKWARD"));

    // Add current state and target state monitoring
    NetworkTableEntry currentStateEntry = superstructureTable.getEntry("Current State");
    NetworkTableEntry targetStateEntry = superstructureTable.getEntry("Target State");
    
    // Update these entries periodically in the periodic method of RobotContainer
    // For now, we'll create them so they appear on the dashboard
    currentStateEntry.setString("Unknown");
    targetStateEntry.setString("Unknown");
  }

  private void RegisterDefaultCommands() {
    // elevator.setDefaultCommand(elevator.defaultElevatorCommand());
    // endEffector.setDefaultCommand(endEffector.defaultEndEffectorCommand());
    // intake.setDefaultCommand(intake.intakeDefault());
  }

  private void BuildDriveTab() {
    ShuffleboardTab testTab = Shuffleboard.getTab("Drive");

    testTab.add("Drivetrain Test", new DrivetrainTest(drive)).withPosition(0, 4).withSize(3, 1);

    testTab.add("Drive Stop", drive.run(drive::stop)).withPosition(3, 4).withSize(2, 1);

    testTab.add("Drive X-Lock", drive.run(drive::stopWithX)).withPosition(5, 4).withSize(2, 1);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.driveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
  }

  private void configureArbitraryTriggers() {
    intake.feederJamTrigger.onTrue(intake.dejamFeeder());
    elevator.elevatorObjectTrigger.onTrue(elevator.dejamElevator());
    intake.rejectCoralTrigger().whileTrue(intake.rejectCoralCommand());
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Command defaultElevatorCommand() {
    return elevator.moveToTargetPosition(
        () -> superstructure.getCurrentState().getElevatorHeight());
  }

  public Command defaultEndEffectorCommand() {
    return endEffector.rotatePivot(() -> superstructure.getCurrentState().getEndEffectorRotation());
  }
}
