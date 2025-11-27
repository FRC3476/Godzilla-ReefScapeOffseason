package frc.robot.subsystems.end_effector;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystems.CanCoderIO;
import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.MotorInputsAutoLogged;
import frc.lib.subsystems.ServoMotorSubsystem;
import frc.lib.subsystems.ServoMotorSubsystemWithCanCoder;
import frc.lib.subsystems.ServoMotorSubsystemWithCanCoderConfig;
import frc.lib.subsystems.ServoMotorSubsystemWithFollowers;
import frc.lib.subsystems.ServoMotorSubsystemWithFollowersConfig;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class EndEffector extends ServoMotorSubsystemWithCanCoder<MotorInputsAutoLogged, MotorIO, CanCoderInputsAutoLogged,
CanCoderIO> {
  private final RobotState robotState;
  private boolean isZeroed = false;
  private static final LoggedTunableNumber pivotTestVolts =
  new LoggedTunableNumber("EndEffector/PivotTestVolts", 1.0);

  public EndEffector(
      ServoMotorSubsystemWithCanCoderConfig config,
      MotorIO io,
      RobotState robotState) {
    super(
        config,
        new MotorInputsAutoLogged(),
        io);
    this.robotState = robotState;
    zeroMagnetOffset(EndEffectorConstants.MIN_ANGLE_ROTATIONS);
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

  public Command moveEndEffectorCommand(DoubleSupplier rotationSupplier, Supplier<Integer> slotSupplier) {
    return motionMagicSetpointCommandBlocking(
      rotationSupplier, EndEffectorConstants.PIVOT_TOLERANCE_ROTATIONS, slotSupplier);
  }

  public Command pivotSTOP() {
    return voltageCommand(() -> 0);
  }

  public Command pivotUP() {
    return voltageCommand(() -> pivotTestVolts.getAsDouble());
  }

  public Command pivotDOWN() {
    return voltageCommand(() -> -pivotTestVolts.getAsDouble());
  }

  public double getMotorVelocityRPS() {
    return getCurrentVelocity() / EndEffectorConstants.EndEffectorConstants2.kEndEffectorConfig.unitToRotorRatio;
  }

  public Command setPivotZero() {
    return Commands.runOnce(()-> 
      {
        zeroMagnetOffset(EndEffectorConstants.MIN_ANGLE_ROTATIONS);
      }
    );
  }
}
