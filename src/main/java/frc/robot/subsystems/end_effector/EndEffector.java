package frc.robot.subsystems.end_effector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {

  private final EndEffectorIO io;
  private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();
  private static EndEffector endEffectorSubsystem;

  public EndEffector(EndEffectorIO io) {
    this.io = io;
    System.out.println("====================EndEffector Subsystem Online====================");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("EndEffector", inputs);
  }

  public double getCurrentPivotPosition() {
    return inputs.pivotData.pivotPosition();
  }

  public Command rotatePivot(DoubleSupplier degreeSupplier) {
    return Commands.run(() -> this.io.setPivotPosition(degreeSupplier.getAsDouble()), this);
  }
}
