package frc.robot.viz;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.climb.ClimbRoller;
import frc.robot.subsystems.climb.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.end_effector.Claw;
import frc.robot.subsystems.end_effector.EndEffector;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LedState;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

// Coral Roller: Red
// Intake Roller: Yellow
// Wrist: Blue
// Elevator: White
// Climber: Yellow
public class RobotViz {
  private final Elevator elevator;
  private final EndEffector endEffector;
  private final Intake intake;
  private final Claw claw;
  private final Feeder feeder;
  private final Climber climber;
  private final ClimbRoller climbRoller;

  private LoggedMechanism2d viz2d = new LoggedMechanism2d(1.27, 2.032);
  private LoggedMechanismRoot2d ledDisplayRoot = viz2d.getRoot("ledDisplay", 0.58, 1.5);
  private LoggedMechanismLigament2d ledDisplayBox =
      new LoggedMechanismLigament2d("LED State", 0.5, 0, 5.0, new Color8Bit(Color.kBlack));

  private Color8Bit blue = new Color8Bit(Color.kBlue);
  private Color8Bit white = new Color8Bit(Color.kWhite);
  private Color8Bit red = new Color8Bit(Color.kRed);
  private Color8Bit yellow = new Color8Bit(Color.kYellow);

  private LoggedMechanismRoot2d driveRoot = viz2d.getRoot("drive", 0.25, 0);
  private LoggedMechanismLigament2d driveLigament =
      new LoggedMechanismLigament2d("drivetrain", 0.76, 0.0, 5.0, blue);

  private final double kElevatorPosX = 0.75;
  private final double kElevatorPosY = 0.51;
  private LoggedMechanismRoot2d elevatorRoot = viz2d.getRoot("elevatorRoot", kElevatorPosX, kElevatorPosY);

  private final double kIntakeRollerPosX = 0.5;
  private final double kIntakeRollerPosY = 0.1;
  private LoggedMechanismRoot2d intakeRoot = viz2d.getRoot("intakeRoot", kIntakeRollerPosX, kIntakeRollerPosY);
  private LoggedMechanismLigament2d intakeLigament =
      new LoggedMechanismLigament2d("intake", 0.3, 110, 2.0, blue);
  private LoggedMechanismLigament2d intakeRollerLigament =
      new LoggedMechanismLigament2d("intakeRoller", 0.051, 180, 9.0, yellow);

  private LoggedMechanismLigament2d elevatorLigament =
      new LoggedMechanismLigament2d("elevator", 0.13, 90.0, 20.0, white);
  private LoggedMechanismLigament2d wristLigament =
      new LoggedMechanismLigament2d("Wrist", 0.38, -180.0, 5.0, blue);
  private LoggedMechanismLigament2d clawRollerLigament =
      new LoggedMechanismLigament2d("clawRoller", 0.051, 90.0, 5.0, red);

  private LoggedMechanismRoot2d indexerRoot = viz2d.getRoot("indexerRoot", 0.63, 0.5);
  private LoggedMechanismLigament2d indexerLigament =
      new LoggedMechanismLigament2d("indexer", 0.051, 0.0, 10.0, red);

  private final double kClimberNormalLength = 0.051;
  private final double kClimberDeployLength = 1.0;
  private LoggedMechanismRoot2d climberRoot = viz2d.getRoot("climberRoot", 0.63, 0.2);
  private LoggedMechanismLigament2d climberLigament =
      new LoggedMechanismLigament2d("climber", kClimberNormalLength, 0.0, 5.0, yellow);

  private Pose3d intakePose3d = new Pose3d();
  private Pose3d elevatorBasePose3d = new Pose3d();
  private Pose3d elevatorMiddlePose3d = new Pose3d();
  private Pose3d elevatorTopPose3d = new Pose3d();
  private Pose3d clawPose3d = new Pose3d();
  private Pose3d climberPose3d = new Pose3d();

  private Color8Bit ledColor = new Color8Bit(0, 0, 0);

  // Track roller rotations for visual effect
  private double accumulatedClawRollerRotations = 0.0;
  private double accumulatedIntakeRollerRotations = 0.0;
  private double accumulatedIndexerRotations = 0.0;
  private double accumulatedClimberRollerRotations = 0.0;
  private double lastUpdateTime = 0.0;

  public Pose3d getClawPose3d() {
    return clawPose3d;
  }

  public RobotViz(
      Elevator elevator,
      EndEffector endEffector,
      Intake intake,
      Claw claw,
      Feeder feeder,
      Climber climber,
      ClimbRoller climbRoller) {
    this.elevator = elevator;
    this.endEffector = endEffector;
    this.intake = intake;
    this.claw = claw;
    this.feeder = feeder;
    this.climber = climber;
    this.climbRoller = climbRoller;

    if (Robot.isReal()) return;
    init2dViz();
  }

  private void init2dViz() {
    viz2d = new LoggedMechanism2d(1.27, 2.032);
    ledDisplayRoot = viz2d.getRoot("ledDisplay", 0.58, 1.5);
    ledDisplayBox = new LoggedMechanismLigament2d("LED State", 0.5, 0, 5.0, new Color8Bit(Color.kBlack));

    driveRoot = viz2d.getRoot("drive", 0.25, 0);
    driveLigament = new LoggedMechanismLigament2d("drivetrain", 0.76, 0.0, 5.0, blue);

    elevatorRoot = viz2d.getRoot("elevatorRoot", kElevatorPosX, kElevatorPosY);
    intakeRoot = viz2d.getRoot("intakeRoot", kIntakeRollerPosX, kIntakeRollerPosY);
    intakeLigament = new LoggedMechanismLigament2d("intake", 0.3, 110, 2.0, blue);
    intakeRollerLigament = new LoggedMechanismLigament2d("intakeRoller", 0.051, 180, 9.0, yellow);

    elevatorLigament = new LoggedMechanismLigament2d("elevator", 0.13, 90.0, 20.0, white);
    wristLigament = new LoggedMechanismLigament2d("Wrist", 0.38, -180.0, 5.0, blue);
    clawRollerLigament = new LoggedMechanismLigament2d("coralRoller", 0.051, 90.0, 5.0, red);

    indexerRoot = viz2d.getRoot("indexerRoot", 0.63, 0.5);
    indexerLigament = new LoggedMechanismLigament2d("indexer", 0.051, 0.0, 10.0, red);

    climberRoot = viz2d.getRoot("climberRoot", 0.63, 0.2);
    climberLigament = new LoggedMechanismLigament2d("climber", kClimberNormalLength, 0.0, 5.0, yellow);

    ledDisplayRoot.append(ledDisplayBox);
    driveRoot.append(this.driveLigament);
    indexerRoot.append(indexerLigament);
    climberRoot.append(climberLigament);

    // Set up elevator and rollers
    elevatorRoot.append(this.elevatorLigament);
    this.elevatorLigament.append(this.wristLigament);
    this.wristLigament.append(this.clawRollerLigament);

    // Intake configuration
    this.intakeRoot.append(this.intakeLigament);
    this.intakeLigament.append(this.intakeRollerLigament);
  }

  public void updateViz() {
    // Update roller rotation tracking based on velocity
    double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    if (lastUpdateTime > 0) {
      double dt = currentTime - lastUpdateTime;
      accumulatedClawRollerRotations += claw.getRollerVelocityRPS() * dt;
      accumulatedIntakeRollerRotations += intake.getRollerVelocityRPS() * dt;
      accumulatedIndexerRotations += feeder.getRightRollerVelocityRPS() * dt;
      accumulatedClimberRollerRotations += climbRoller.getRollerVelocityRPS() * dt;
    }
    lastUpdateTime = currentTime;

    // Get current mechanism states
    double wristRadians = getWristRadians();
    double elevatorHeightM = getElevatorHeightMeters();
    double intakePivotRadians = getIntakePivotRadians();

    // Get current roller rotations
    double clawRollerRotations = accumulatedClawRollerRotations;
    double intakeRollerRotations = accumulatedIntakeRollerRotations;
    double indexerRotations = accumulatedIndexerRotations;
    double climberRollerRotations = accumulatedClimberRollerRotations;
    double climberPivotRadians = getClimberPivotRadians();

    // Update elevator stages based on height
    double baseHeight = 0.0;
    double middleHeight = Math.max(0.0, elevatorHeightM - 0.631825);
    double topHeight = elevatorHeightM;

    intakePose3d = new Pose3d(0, 0, 0, new Rotation3d(0, -intakePivotRadians, 0));
    elevatorBasePose3d = new Pose3d(0, 0, baseHeight, new Rotation3d(0, 0, 0));
    elevatorMiddlePose3d = new Pose3d(0, 0, middleHeight, new Rotation3d(0, 0, 0));
    elevatorTopPose3d = new Pose3d(0, 0, topHeight, new Rotation3d(0, 0, 0));
    climberPose3d = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

    double intakeX = -0.29;
    double intakeZ = 0.215;
    double clawX = 0.18;
    double clawZ = 0.69;
    double climberY = 0.355;
    double climberZ = 0.405;

    // Rotate claw based on wrist angle
    clawPose3d =
        new Pose3d(
            clawX,
            0,
            clawZ + elevatorHeightM,
            new Rotation3d(0, -wristRadians - Units.degreesToRadians(90), 0));
    climberPose3d = new Pose3d(0, climberY, climberZ, new Rotation3d(-climberPivotRadians, 0, 0));
    intakePose3d = new Pose3d(intakeX, 0, intakeZ, new Rotation3d(0, -intakePivotRadians, 0));

    Logger.recordOutput(
        "ComponentsPoseArray",
        new Pose3d[] {
          intakePose3d,
          elevatorBasePose3d,
          elevatorMiddlePose3d,
          elevatorTopPose3d,
          clawPose3d,
          climberPose3d
        });

    LedState currentLEDState = RobotState.getLedState();
    ledColor = new Color8Bit(currentLEDState.red, currentLEDState.green, currentLEDState.blue);
    Logger.recordOutput(
        "LEDState",
        String.format(
            "R:%d G:%d B:%d", currentLEDState.red, currentLEDState.green, currentLEDState.blue));

    if (Robot.isReal()) return;

    // Update 2D visualization
    this.elevatorRoot.setPosition(kElevatorPosX, kElevatorPosY + elevatorHeightM);
    this.intakeLigament.setAngle(180.0 + Units.radiansToDegrees(intakePivotRadians));
    this.intakeRollerLigament.setAngle(Units.rotationsToDegrees(-intakeRollerRotations));
    this.climberLigament.setAngle(Units.rotationsToDegrees(climberRollerRotations));

    if (climberPivotRadians > Math.toRadians(45.0)) {
      this.climberLigament.setLength(kClimberDeployLength);
    } else {
      this.climberLigament.setLength(kClimberNormalLength);
    }

    this.wristLigament.setAngle(Math.toDegrees(wristRadians) - 90);
    this.clawRollerLigament.setAngle(Units.rotationsToDegrees(clawRollerRotations));
    this.indexerLigament.setAngle(Units.rotationsToDegrees(indexerRotations));

    Logger.recordOutput("viz2d", this.viz2d);
    ledDisplayBox.setColor(ledColor);
  }

  // State getter methods with unit conversions
  private double getElevatorHeightMeters() {
    return Units.inchesToMeters(elevator.getCurrentPosition());
  }

  private double getWristRadians() {
    return Units.rotationsToRadians(endEffector.getCurrentPivotPosition());
  }

  private double getIntakePivotRadians() {
    return Units.rotationsToRadians(intake.getCurrentPivotPosition());
  }

  private double getClimberPivotRadians() {
    return climber.getPositionRadians();
  }
}

