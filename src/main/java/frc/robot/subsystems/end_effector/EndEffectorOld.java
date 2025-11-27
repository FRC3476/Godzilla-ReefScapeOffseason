package frc.robot.subsystems.end_effector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RobotTime;
import frc.robot.util.Util;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class EndEffectorOld extends SubsystemBase {

  private final EndEffectorIO io;
  private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

  private double pivotSetpoint;

  private static final LoggedTunableNumber pivotTestVolts =
      new LoggedTunableNumber("EndEffector/PivotTestVolts", 1.0);

  // Tunable numbers for manual testing
  private static final LoggedTunableNumber pivotKP =
      new LoggedTunableNumber("EndEffector/PivotKP", EndEffectorConstants.Tunable_PIVOT_kP);
  private static final LoggedTunableNumber pivotKI =
      new LoggedTunableNumber("EndEffector/PivotKI", EndEffectorConstants.Tunable_PIVOT_kI);
  private static final LoggedTunableNumber pivotKD =
      new LoggedTunableNumber("EndEffector/PivotKD", EndEffectorConstants.Tunable_PIVOT_kD);
  private static final LoggedTunableNumber pivotKG =
      new LoggedTunableNumber("EndEffector/PivotKG", EndEffectorConstants.Tunable_PIVOT_kG);
  private static final LoggedTunableNumber pivotKS =
      new LoggedTunableNumber("EndEffector/PivotKS", EndEffectorConstants.Tunable_PIVOT_kS);
  private static final LoggedTunableNumber pivotKV =
      new LoggedTunableNumber("EndEffector/PivotKV", EndEffectorConstants.Tunable_PIVOT_kV);
  private static final LoggedTunableNumber pivotKA =
      new LoggedTunableNumber("EndEffector/PivotKA", EndEffectorConstants.Tunable_PIVOT_kA);
  private static final LoggedTunableNumber pivotVelo =
      new LoggedTunableNumber("EndEffector/PivotVelo", EndEffectorConstants.Tunable_PIVOT_Velo);
  private static final LoggedTunableNumber pivotAccel =
      new LoggedTunableNumber("EndEffector/PivotAccel", EndEffectorConstants.Tunable_PIVOT_Accel);
  private static final LoggedTunableNumber pivotJerk =
      new LoggedTunableNumber("EndEffector/PivotJerk", EndEffectorConstants.Tunable_PIVOT_Jerk);

  public EndEffectorOld(EndEffectorIO io) {
    this.io = io;
    System.out.println("====================EndEffector Subsystem Online====================");
  }

  @Override
  public void periodic() {
    double timestamp = RobotTime.getTimestampSeconds();
    io.updateInputs(inputs);
    Logger.processInputs("EndEffector", inputs);

    // Update PID/FF values if they have changed
    if (pivotKP.hasChanged(hashCode())
        || pivotKI.hasChanged(hashCode())
        || pivotKD.hasChanged(hashCode())
        || pivotKG.hasChanged(hashCode())
        || pivotKS.hasChanged(hashCode())
        || pivotKV.hasChanged(hashCode())
        || pivotKA.hasChanged(hashCode())
        || pivotVelo.hasChanged(hashCode())
        || pivotAccel.hasChanged(hashCode())
        || pivotJerk.hasChanged(hashCode())) {
      io.updatePivotPIDFF(
          pivotKP.get(),
          pivotKI.get(),
          pivotKD.get(),
          pivotKG.get(),
          pivotKS.get(),
          pivotKV.get(),
          pivotKA.get(),
          pivotVelo.get(),
          pivotAccel.get(),
          pivotJerk.get());
    }

    Logger.recordOutput(
        getName() + "/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
    Logger.recordOutput(
        "EndEffector/currentCommand",
        (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());
    Logger.recordOutput("EndEffector/TargetPos", pivotSetpoint);
  }

  public double getCurrentPivotPosition() {
    return inputs.pivotData.pivotPosition();
  }

  @AutoLogOutput(key = "EndEffector/Pivot/InTolerance")
  public boolean isPivotInTolerance() {
    return MathUtil.isNear(
        pivotSetpoint,
        inputs.pivotData.pivotPosition(),
        EndEffectorConstants.PIVOT_TOLERANCE_ROTATIONS);
  }

  // Normal tolerance
  public Command moveEndEffectorCommand(
      DoubleSupplier rotationsSupplier, Supplier<Integer> slotSupplier) {
    return Commands.sequence(
        this.rotatePivotCommand(rotationsSupplier, slotSupplier),
        this.waitUntilTargetPositionCommand());
  }
  // Normal tolerance
  public Command moveEndEffectorCommand(DoubleSupplier rotationsSupplier) {
    return Commands.sequence(
        this.rotatePivotCommand(rotationsSupplier), this.waitUntilTargetPositionCommand());
  }

  public Command rotatePivotCommand(
      DoubleSupplier rotationSupplier, Supplier<Integer> slotSupplier) {
    pivotSetpoint =
        MathUtil.clamp(
            rotationSupplier.getAsDouble(),
            EndEffectorConstants.MIN_ANGLE_ROTATIONS,
            EndEffectorConstants.MAX_ANGLE_ROTATIONS);
    return Commands.runOnce(
        () -> this.io.setPivotPosition(() -> pivotSetpoint, slotSupplier), this);
  }

  public Command rotatePivotCommand(DoubleSupplier rotationSupplier) {
    return rotatePivotCommand(rotationSupplier, () -> 0);
  }

  public Command waitUntilTargetPositionCommand() {
    return Commands.waitUntil(() -> isPivotInTolerance());
  }

  public Command pivotUP() {
    return Commands.run(() -> this.io.setPivotVoltage(pivotTestVolts.get()), this);
  }

  public Command pivotDOWN() {
    return Commands.run(() -> this.io.setPivotVoltage(-pivotTestVolts.get()), this);
  }

  public Command pivotSTOP() {
    return Commands.run(() -> this.io.setPivotVoltage(0), this);
  }

  public Command setPivotZero() {
    return Commands.runOnce(() -> this.io.setPivotZero(), this);
  }

  public boolean isPivotSafe() {
    return Util.inRange(
        inputs.pivotData.pivotPosition(),
        EndEffectorConstants.MIN_SAFE_ANGLE_ROTATIONS,
        EndEffectorConstants.MAX_SAFE_ANGLE_ROTATIONS);
  }
}
