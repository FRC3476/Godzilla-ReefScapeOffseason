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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.arbitraryTriggers.ArbitraryTriggers;
import frc.robot.auto.AutoChooserSetup;
import frc.robot.auto.NamedCommandsSetup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EndEffectorConstants.ClawState;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Field.FieldConstants;
import frc.robot.Field.FieldUtils;
import frc.robot.RobotState.AlgaeIntake;
import frc.robot.RobotState.CoralBranch;
import frc.robot.RobotState.ScoreLevel;
import frc.robot.RobotState.ScorePosition;
import frc.robot.commands.DriveCommands;
import frc.robot.humanControls.DriverControls;
import frc.robot.humanControls.ElasticTabs;
import frc.robot.humanControls.OperatorControls;
import frc.robot.commands.DriveToCoralCommand;
import frc.robot.commands.DriveToPosePIDCommand;
import frc.robot.commands.GarageDriveToPoseCommand;
import frc.robot.commands.ParallelDriveCommand;
import frc.robot.commands.PathfindToPoseCommand;
import frc.robot.commands.Rumble;
import frc.robot.commands.Score;
import frc.robot.subsystems.climb.ClimbRoller;
import frc.robot.subsystems.climb.ClimbRollerIO;
import frc.robot.subsystems.climb.ClimbRollerIOReal;
import frc.robot.subsystems.climb.ClimbRollerIOSim;
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
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionFieldPoseEstimate;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOHardwareLimelight;
import frc.robot.subsystems.vision.VisionIOSimPhoton;
import frc.robot.util.Controls.StreamDeck.StreamDeck;
import java.util.function.Consumer;

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
  private final ClimbRoller climbRoller;
  private final Feeder feeder;
  private final Vision vision;

  private final Consumer<VisionFieldPoseEstimate> visionEstimateConsumer =
      new Consumer<VisionFieldPoseEstimate>() {
        @Override
        public void accept(VisionFieldPoseEstimate estimate) {
          drive.addVisionMeasurement(estimate);
        }
      };

  private final RobotState robotState = new RobotState(visionEstimateConsumer, this);

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final StreamDeck streamdeck = new StreamDeck();
  // private DriverControls driverControls;
  // private OperatorControls operatorControls;
  // private ElasticTabs elasticTabs;
  // private ArbitraryTriggers arbitraryTriggers;
  // private NamedCommandsSetup namedCommands;
  private AutoChooserSetup autoChooserSetup;

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
        climbRoller = new ClimbRoller(new ClimbRollerIOReal());
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
        climbRoller = new ClimbRoller(new ClimbRollerIOSim());
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
        climbRoller = new ClimbRoller(new ClimbRollerIO() {});
        vision = new Vision(new VisionIO() {}, robotState);
        drive = new DriveSubsystem(new DriveIO() {}, robotState);
        break;
    }

    // ============================================================================================================================================

    // Setup named commands for auto
    // namedCommands = new NamedCommandsSetup(this, robotState);
    new NamedCommandsSetup(this, robotState);
    // Set up auto routines
    autoChooserSetup = new AutoChooserSetup();

    // Configure default commands for subsystems
    RegisterDefaultCommands();

    // Build elastic tabs for testing
    new ElasticTabs(this, robotState);

    // Configure the button bindings
    configureButtonBindings();

    // Configure arbitrary triggers
    new ArbitraryTriggers(this, controller, robotState);
  }

  private void configureButtonBindings() {
    new DriverControls(this, controller, robotState);
    new OperatorControls(this, streamdeck, robotState);
    // new TestOperatorControls(this, streamdeck, robotState);
  }

  private void RegisterDefaultCommands() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX() * Math.abs(controller.getRightX())));
    claw.setDefaultCommand(claw.clawDefault());
    intake.setDefaultCommand(intake.intakeDefault());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooserSetup.getAutonomousCommand();
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

  public Claw getClaw() {
    return claw;
  }

  public Feeder getFeeder() {
    return feeder;
  }

  public Climber getClimber() {
    return climber;
  }

  public ClimbRoller getClimbRoller() {
    return climbRoller;
  }

  public DriveSubsystem getDrive() {
    return drive;
  }

  public Superstructure getSuperStructure() {
    return superstructure;
  }

  public Vision getVision() {
    return vision;
  }
}
