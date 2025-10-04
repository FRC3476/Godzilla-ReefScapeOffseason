package frc.robot.subsystems.end_effector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {

  private final EndEffectorIO io;
  private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

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
  private static final LoggedTunableNumber pivotVelo =
      new LoggedTunableNumber("EndEffector/PivotVelo", EndEffectorConstants.Tunable_PIVOT_Velo);
  private static final LoggedTunableNumber pivotAccel =
      new LoggedTunableNumber("EndEffector/PivotAccel", EndEffectorConstants.Tunable_PIVOT_Accel);
  private static final LoggedTunableNumber pivotJerk =
      new LoggedTunableNumber("EndEffector/PivotJerk", EndEffectorConstants.Tunable_PIVOT_Jerk);

  public EndEffector(EndEffectorIO io) {
    this.io = io;
    System.out.println("====================EndEffector Subsystem Online====================");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("EndEffector", inputs);

    // Update PID/FF values if they have changed
    if (pivotKP.hasChanged(hashCode())
        || pivotKI.hasChanged(hashCode())
        || pivotKD.hasChanged(hashCode())
        || pivotKG.hasChanged(hashCode())
        || pivotKS.hasChanged(hashCode())
        || pivotVelo.hasChanged(hashCode())
        || pivotAccel.hasChanged(hashCode())
        || pivotJerk.hasChanged(hashCode())) {
      io.updatePivotPIDFF(
          pivotKP.get(),
          pivotKI.get(),
          pivotKD.get(),
          pivotKG.get(),
          pivotKS.get(),
          pivotVelo.get(),
          pivotAccel.get(),
          pivotJerk.get());
    }
  }

  public double getCurrentPivotPosition() {
    return inputs.pivotData.pivotPosition();
  }

  public Command rotatePivot(DoubleSupplier radianSupplier) {
    return Commands.runOnce(() -> this.io.setPivotPosition(radianSupplier.getAsDouble()), this);
  }

  public Command pivotUP() {
    return Commands.run(() -> this.io.setPivotVoltage(pivotTestVolts.get()), this);
  }

  public Command pivotDOWN() {
    return Commands.run(() -> this.io.setPivotVoltage(-pivotTestVolts.get()), this);
  }

  public Command pivotSTOP() {
    return Commands.runOnce(() -> this.io.setPivotVoltage(0), this);
  }

  public Command setPivotZero() {
    return Commands.runOnce(() -> this.io.setPivotZero(), this);
  }

  public double getCurrentAngle() {
    return inputs.pivotData.pivotPosition();
  }

  public boolean isInTolerance(double targetAngle, double toleranceRad) {
    return Math.abs(getCurrentAngle() - targetAngle) <= toleranceRad;
  }

  public boolean isInSafeAngleRange() {
    double currentAngle = getCurrentAngle();
    return currentAngle >= frc.robot.Constants.EndEffectorConstants.SAFE_ANGLE_LOWER_RAD
        && currentAngle <= frc.robot.Constants.EndEffectorConstants.SAFE_ANGLE_UPPER_RAD;
  }
}
