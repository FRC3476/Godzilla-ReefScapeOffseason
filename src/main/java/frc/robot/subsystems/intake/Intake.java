package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  public boolean isPivotAtSetpoint() {
    return Math.abs(inputs.pivotData.positionRad() - frc.robot.Constants.IntakeConstants.PIVOT_L1_SETPOINT_RAD) < frc.robot.Constants.IntakeConstants.PIVOT_TOLERANCE_RAD;
  }

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private static final LoggedTunableNumber rollerIntakeVolts =
      new LoggedTunableNumber("Intake/RollerVolts", 12.0);

    public Command scoreIntakeL1() {
      return Commands.sequence(
        Commands.runOnce(() -> io.setPivotPosition(frc.robot.Constants.IntakeConstants.PIVOT_L1_SETPOINT_RAD), this),
        Commands.waitUntil(this::isPivotAtSetpoint),
        Commands.runOnce(() -> io.setRollerVoltage(frc.robot.Constants.IntakeConstants.ROLLER_L1_SETPOINT_VOLTS), this)
      );
    }

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public boolean isCoralInIntake() {
    return inputs.canRangeData.tripped() && inputs.canRangeData.isSensorConnected();
  }

  public Command intakeFWD() {
    return Commands.run(() -> this.io.setRollerVoltage(rollerIntakeVolts.get()), this);
  }

  public Command intakeRVS() {
    return Commands.run(() -> this.io.setRollerVoltage(-rollerIntakeVolts.get()), this);
  }

  public Command intakeSTOP() {
    return Commands.run(() -> this.io.setRollerVoltage(0), this);
  }
}
