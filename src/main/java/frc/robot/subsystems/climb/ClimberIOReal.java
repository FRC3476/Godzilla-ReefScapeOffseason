package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class ClimberIOReal implements ClimberIO {

  // Hardware
  private final TalonFX talon;

  // Status Signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> torqueCurrentAmps;
  private final StatusSignal<Temperature> temp;

  // control requests
  private final VoltageOut voltsRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  public ClimberIOReal() {
    talon = new TalonFX(Constants.ClimbConstants.ID);

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrentAmps = talon.getSupplyCurrent();
    torqueCurrentAmps = talon.getTorqueCurrent();
    temp = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, supplyCurrentAmps, torqueCurrentAmps, temp);
    PhoenixUtil.registerSignals(
        false, position, velocity, appliedVolts, supplyCurrentAmps, torqueCurrentAmps, temp);
    talon.optimizeBusUtilization();
  }

  public void updateInputs(ClimberIOInputs inputs) {
    inputs.data =
        new ClimberIOData(
            BaseStatusSignal.isAllGood(
                position, velocity, appliedVolts, supplyCurrentAmps, torqueCurrentAmps, temp),
            position.getValueAsDouble(),
            velocity.getValueAsDouble(),
            appliedVolts.getValueAsDouble(),
            supplyCurrentAmps.getValueAsDouble(),
            torqueCurrentAmps.getValueAsDouble(),
            temp.getValueAsDouble());
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltsRequest.withOutput(volts));
  }
}
