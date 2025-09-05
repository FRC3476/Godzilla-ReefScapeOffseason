package frc.robot.subsystems.end_effector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {

  private EndEffectorIO io;
  private EndEffectorIOInputsAutoLogged inputs;
  private static EndEffector endEffectorSubsystem;

  private double rollerSpeed;

  public static EndEffector getInstance() {
    if (endEffectorSubsystem == null) {
      endEffectorSubsystem = new EndEffector(new EndEffectorIOReal());
    }
    return endEffectorSubsystem;
  }

  public EndEffector(EndEffectorIO io) {
    this.io = io;
    System.out.println("====================EndEffector Subsystem Online====================");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("EndEffector", inputs);
    Logger.recordOutput("EndEffector/TargetRollerSpeed", rollerSpeed);
  }

  public void setRollerVelocity(double velocity) {
    rollerSpeed = velocity;
    io.setRollerVelocity(velocity);
  }

  public void setRollerVoltage(double voltage) {
    io.setRollerVoltage(voltage);
  }
}
