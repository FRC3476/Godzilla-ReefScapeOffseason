package frc.robot.subsystems.end_effector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
      new LoggedTunableNumber("EndEffector/PivotKP", 0.0);
  private static final LoggedTunableNumber pivotKI =
      new LoggedTunableNumber("EndEffector/PivotKI", 0.0);
  private static final LoggedTunableNumber pivotKD =
      new LoggedTunableNumber("EndEffector/PivotKD", 0.0);
  private static final LoggedTunableNumber pivotKG =
      new LoggedTunableNumber("EndEffector/PivotKG", 0.0);
  private static final LoggedTunableNumber pivotKS =
      new LoggedTunableNumber("EndEffector/PivotKS", 0.0);
  private static final LoggedTunableNumber pivotVelo =
      new LoggedTunableNumber("EndEffector/PivotVelo", 0.0);
  private static final LoggedTunableNumber pivotAccel =
      new LoggedTunableNumber("EndEffector/PivotAccel", 0.0);
  private static final LoggedTunableNumber pivotJerk =
      new LoggedTunableNumber("EndEffector/PivotJerk", 0.0);

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
    return Commands.runOnce(() -> this.io.setPivotVoltage(pivotTestVolts.get()), this);
  }

  public Command pivotDOWN() {
    return Commands.runOnce(() -> this.io.setPivotVoltage(-pivotTestVolts.get()), this);
  }

  public Command pivotSTOP() {
    return Commands.runOnce(() -> this.io.setPivotVoltage(0), this);
  }

  public Command setPivotZero() {
    return Commands.runOnce(() -> this.io.setPivotZero(), this);
  }
}
