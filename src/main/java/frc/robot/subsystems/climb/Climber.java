package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  
  // Tunable numbers for manual testing and gravity compensation
  private static final LoggedTunableNumber climberIntakeVolts =
      new LoggedTunableNumber("Climber/DeployVolts", 0);
  private static final LoggedTunableNumber climberManualUpVolts =
      new LoggedTunableNumber("Climber/ManualUpVolts", 6.0);
  private static final LoggedTunableNumber climberManualDownVolts =
      new LoggedTunableNumber("Climber/ManualDownVolts", -2.0);
  private static final LoggedTunableNumber climberKG =
      new LoggedTunableNumber("Climber/kG", 0.0);
  private static final LoggedTunableNumber climberDirection =
      new LoggedTunableNumber("Climber/Direction", 1.0); // 1.0 for normal, -1.0 for inverted

  private static Climber climberSubsystem;

  public static Climber getInstance() {
    if (climberSubsystem == null) {
      climberSubsystem = new Climber(new ClimberIOReal());
    }
    return climberSubsystem;
  }

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    
    // Log tunable values for debugging
    Logger.recordOutput("Climber/kG", climberKG.get());
    Logger.recordOutput("Climber/Direction", climberDirection.get());
    Logger.recordOutput("Climber/ManualUpVolts", climberManualUpVolts.get());
    Logger.recordOutput("Climber/ManualDownVolts", climberManualDownVolts.get());
  }

  // Original commands (kept for compatibility)
  public Command climbDeploy() {
    return Commands.run(() -> this.io.runVolts(climberIntakeVolts.get() * climberDirection.get()), this);
  }

  public Command climbSTOP() {
    return Commands.run(() -> this.io.runVolts(climberKG.get() * climberDirection.get()), this);
  }

  // Manual test functions for low voltage testing and tuning
  public Command climberManualUp() {
    return Commands.run(() -> this.io.runVolts((climberManualUpVolts.get() + climberKG.get()) * climberDirection.get()), this);
  }

  public Command climberManualDown() {
    return Commands.run(() -> this.io.runVolts((climberManualDownVolts.get() + climberKG.get()) * climberDirection.get()), this);
  }

  public Command climberHold() {
    return Commands.run(() -> this.io.runVolts(climberKG.get() * climberDirection.get()), this);
  }

  // Utility methods for getting current state
  public double getCurrentPosition() {
    return inputs.data.positionRads();
  }

  public double getCurrentVelocity() {
    return inputs.data.velocityRadsPerSec();
  }

  public double getAppliedVoltage() {
    return inputs.data.appliedVoltage();
  }

  public double getTorqueCurrent() {
    return inputs.data.torqueCurrentAmps();
  }

  public boolean isMotorConnected() {
    return inputs.data.motorConnected();
  }
}
