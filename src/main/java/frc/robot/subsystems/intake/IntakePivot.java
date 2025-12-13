package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.canDevice.CanCoderIO;
import frc.lib.subsystems.real.ServoMotorSubsystemWithCanCoder;
import frc.lib.subsystems.real.ServoMotorSubsystemWithCanCoderConfig;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.RobotState;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.superstructure.CoralStateTracker;
import frc.robot.subsystems.superstructure.CoralStateTracker.CoralPosition;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RobotTime;
import org.littletonrobotics.junction.Logger;

public class IntakePivot
    extends ServoMotorSubsystemWithCanCoder<
        MotorInputsAutoLogged, MotorIO, CanCoderInputsAutoLogged, CanCoderIO> {
  private final RobotState robotState;
  // Tunable numbers for manual testing
  private static final LoggedTunableNumber pivotKP =
      new LoggedTunableNumber("Intake/PivotKP", IntakeConstants.Tuneable_pivotKP);
  private static final LoggedTunableNumber pivotKI =
      new LoggedTunableNumber("Intake/PivotKI", IntakeConstants.Tuneable_pivotKI);
  private static final LoggedTunableNumber pivotKD =
      new LoggedTunableNumber("Intake/PivotKD", IntakeConstants.Tuneable_pivotKD);
  private static final LoggedTunableNumber pivotKG =
      new LoggedTunableNumber("Intake/PivotKG", IntakeConstants.Tuneable_pivotKG);
  private static final LoggedTunableNumber pivotKS =
      new LoggedTunableNumber("Intake/PivotKS", IntakeConstants.Tuneable_pivotKS);
  private static final LoggedTunableNumber pivotVelo =
      new LoggedTunableNumber("Intake/PivotVelo", IntakeConstants.Tuneable_pivot_ACCEL);
  private static final LoggedTunableNumber pivotAccel =
      new LoggedTunableNumber("Intake/PivotAccel", IntakeConstants.Tuneable_pivot_VELOCITY);
  private static final LoggedTunableNumber pivotJerk =
      new LoggedTunableNumber("Intake/PivotJerk", IntakeConstants.Tuneable_pivotJERK);

  private static final LoggedTunableNumber pivotManualTestVolts =
      new LoggedTunableNumber("Intake/PivotManualTestVolts", 1.0);

  public IntakePivot(
      ServoMotorSubsystemWithCanCoderConfig config,
      MotorIO io,
      CanCoderIO canCoderIO,
      RobotState robotState) {
      super(config, new MotorInputsAutoLogged(), io, new CanCoderInputsAutoLogged(), canCoderIO);
      this.robotState = robotState;
      zeroMagnetOffset(IntakeConstants.PIVOT_UP_POSITION);
      setDefaultCommand(
        motionMagicSetpointCommand(this::getPositionSetpointUnits)
            .withName(getName() + " Default Command Hold Position")
            .ignoringDisable(true));
      }
  
  @Override
  public void periodic() {
    super.periodic();

    Logger.recordOutput(getName() + "/positionRotations", getCurrentPosition());
  }

  public Command setPivotDown() {
    return motionMagicSetpointCommand(() -> IntakeConstants.PIVOT_INTAKE_POSITION);
  }

  public Command setPivotUp() {
    return motionMagicSetpointCommand(() -> IntakeConstants.PIVOT_UP_POSITION);
  }
  
  public Command setPivotScoring() {
    return motionMagicSetpointCommand(() -> IntakeConstants.PIVOT_SCORING_POSITION);
  }
  
  public Command zeroPivotAtPivotUp() {
    return runOnce(() -> zeroMagnetOffset(IntakeConstants.PIVOT_UP_POSITION));
  }
  
  public Command pivotManualTestForward() {
    return voltageCommand(pivotManualTestVolts::get);
  }

  public Command pivotManualTestReverse() {
    return voltageCommand(() -> -pivotManualTestVolts.get());
  }

  public Command pivotStop() {
    return voltageCommand(() -> 0.0);
  }
}
