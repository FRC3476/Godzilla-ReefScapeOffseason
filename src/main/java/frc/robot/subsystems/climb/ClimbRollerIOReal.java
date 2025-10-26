package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.util.CANDiagnostics;
import frc.robot.util.MotorStallDetection;
import frc.robot.util.PhoenixUtil;

public class ClimbRollerIOReal implements ClimbRollerIO {

  protected TalonFX rollerTalonFX;

  private TorqueCurrentFOC roller_c_request = new TorqueCurrentFOC(0);
  private VoltageOut roller_m_request = new VoltageOut(0).withEnableFOC(true);

  StatusSignal<AngularVelocity> rollerVelocityRPS;
  StatusSignal<Voltage> rollerAppliedVolts;
  StatusSignal<Current> rollerStatorCurrentAmps;
  StatusSignal<Current> rollerSupplyCurrentAmps;
  StatusSignal<Temperature> rollerTempCelsius;

  private final BaseStatusSignal[] signals;

  public ClimbRollerIOReal() {
    rollerTalonFX = new TalonFX(ClimbConstants.rollerID, Constants.DRIVE_CANIVORE);
    PhoenixUtil.tryUntilOk(
        5, () -> rollerTalonFX.getConfigurator().apply(ClimbConstants.ROLLER_TALON_CONFIG));

    rollerVelocityRPS = rollerTalonFX.getRotorVelocity();
    rollerAppliedVolts = rollerTalonFX.getMotorVoltage();
    rollerStatorCurrentAmps = rollerTalonFX.getStatorCurrent();
    rollerSupplyCurrentAmps = rollerTalonFX.getSupplyCurrent();
    rollerTempCelsius = rollerTalonFX.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        rollerVelocityRPS,
        rollerAppliedVolts,
        rollerStatorCurrentAmps,
        rollerSupplyCurrentAmps,
        rollerTempCelsius);
    ParentDevice.optimizeBusUtilizationForAll(rollerTalonFX);

    signals =
        new BaseStatusSignal[] {
          rollerVelocityRPS,
          rollerAppliedVolts,
          rollerStatorCurrentAmps,
          rollerSupplyCurrentAmps,
          rollerTempCelsius
        };
  }

  @Override
  public void updateInputs(ClimbRollerIOInputs inputs) {
    BaseStatusSignal.refreshAll(signals);
    CANDiagnostics.checkSignalHealth("ClimbRoller", signals);

    inputs.rollerData =
        new ClimbRollerData(
            BaseStatusSignal.isAllGood(
                rollerVelocityRPS,
                rollerAppliedVolts,
                rollerStatorCurrentAmps,
                rollerSupplyCurrentAmps,
                rollerTempCelsius),
            rollerVelocityRPS.getValueAsDouble(),
            rollerAppliedVolts.getValueAsDouble(),
            rollerStatorCurrentAmps.getValueAsDouble(),
            rollerSupplyCurrentAmps.getValueAsDouble(),
            rollerTempCelsius.getValueAsDouble());
  }

  public void setTorqueCurrent(double amps) {
    rollerTalonFX.setControl(roller_c_request.withOutput(amps));
  }

  @Override
  public void setRollerVoltage(double voltage) {
    rollerTalonFX.setControl(roller_m_request.withOutput(voltage));
  }

  @Override
  public boolean checkRollerStalled() {
    return MotorStallDetection.isMotorStalled(
        rollerStatorCurrentAmps.getValueAsDouble(),
        rollerVelocityRPS.getValueAsDouble(),
        ClimbConstants.ROLLER_STALLED_CURRENT,
        ClimbConstants.ROLLER_STALLED_RPS); // TODO fine-adjust stalled rps and current in constants
  }
}
