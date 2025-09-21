package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import static edu.wpi.first.units.Units.Volts;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private static final LoggedTunableNumber climberIntakeVolts =
      new LoggedTunableNumber("ClimberVolts", 0);

  private static Climber climberSubsystem;

  // SysId routine for characterization
  private final SysIdRoutine climberSysId;

  public static Climber getInstance() {
    if (climberSubsystem == null) {
      climberSubsystem = new Climber(new ClimberIOReal());
    }
    return climberSubsystem;
  }

  public Climber(ClimberIO io) {
    this.io = io;
    
    // Configure SysId routine for climb motor
    climberSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                state -> Logger.recordOutput("Climber/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> io.runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public Command climbMoveToTargetPosition(double position) {
    return Commands.run(() -> this.io.setClimbPosition(position), this);
  }

  public Command climbDeploy() {
    return Commands.run(() -> this.io.runVolts(climberIntakeVolts.get()), this);
  }

  public Command climbSTOP() {
    return Commands.run(() -> this.io.runVolts(0), this);
  }

  // SysId characterization commands
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return climberSysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return climberSysId.dynamic(direction);
  }
}
