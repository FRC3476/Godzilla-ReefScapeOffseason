package frc.robot.subsystems.end_effector;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.util.PhoenixUtil;

public class EndEffectorIOReal implements EndEffectorIO {

  private TalonFX pivotTalonFX;

  private MotionMagicVoltage pivot_m_request =
      new MotionMagicVoltage(PhysicalConstants.ABSOLUTE_ZERO).withEnableFOC(true);

  private VoltageOut roller_m_request =
      new VoltageOut(PhysicalConstants.ABSOLUTE_ZERO).withEnableFOC(true);

  private VoltageOut pivotVoltageRequest =
      new VoltageOut(PhysicalConstants.ABSOLUTE_ZERO).withEnableFOC(true);

  private final BaseStatusSignal[] signals;

  // =====Logged Values=====
  StatusSignal<Angle> pivotPosition;
  StatusSignal<Voltage> pivotAppliedVolts;
  StatusSignal<Current> pivotTorqueCurrentAmps;
  StatusSignal<Current> pivotSupplyCurrentAmps;
  StatusSignal<Temperature> pivotTempCelsius;

  public EndEffectorIOReal() {
    pivotTalonFX = new TalonFX(EndEffectorConstants.pivotID, Constants.misc_canivore);
    PhoenixUtil.tryUntilOk(
        5, () -> pivotTalonFX.getConfigurator().apply(EndEffectorConstants.PIVOT_TALON_CONFIG));
    pivotPosition = pivotTalonFX.getPosition();
    pivotAppliedVolts = pivotTalonFX.getMotorVoltage();
    pivotTorqueCurrentAmps = pivotTalonFX.getTorqueCurrent();
    pivotSupplyCurrentAmps = pivotTalonFX.getSupplyCurrent();
    pivotTempCelsius = pivotTalonFX.getDeviceTemp();

    signals =
        new BaseStatusSignal[] {
          pivotPosition,
          pivotAppliedVolts,
          pivotTorqueCurrentAmps,
          pivotSupplyCurrentAmps,
          pivotTempCelsius
        };

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        pivotPosition,
        pivotAppliedVolts,
        pivotTorqueCurrentAmps,
        pivotSupplyCurrentAmps,
        pivotTempCelsius);
    ParentDevice.optimizeBusUtilizationForAll(pivotTalonFX);
    PhoenixUtil.registerSignals(
        true,
        pivotPosition,
        pivotAppliedVolts,
        pivotTorqueCurrentAmps,
        pivotSupplyCurrentAmps,
        pivotTempCelsius);
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    BaseStatusSignal.refreshAll(signals);
    inputs.pivotData =
        new EE_PivotData(
            BaseStatusSignal.isAllGood(
                pivotPosition,
                pivotAppliedVolts,
                pivotTorqueCurrentAmps,
                pivotSupplyCurrentAmps,
                pivotTempCelsius),
            Units.rotationsToRadians(pivotPosition.getValueAsDouble()),
            pivotAppliedVolts.getValueAsDouble(),
            pivotTorqueCurrentAmps.getValueAsDouble(),
            pivotSupplyCurrentAmps.getValueAsDouble(),
            pivotTempCelsius.getValueAsDouble());
  }

  @Override
  public void setPivotVoltage(double voltage) {
    pivotTalonFX.setControl(pivotVoltageRequest.withOutput(voltage));
  }

  @Override
  public void setPivotPosition(double position) {
    pivotTalonFX.setControl(pivot_m_request.withPosition(Rotation.convertFrom(position, Degree)));
  }

  @Override
  public void updatePivotPIDFF(double kP, double kI, double kD, double kG, double kS) {
    var pivotConfig = new TalonFXConfiguration();
    pivotTalonFX.getConfigurator().refresh(pivotConfig);
    pivotConfig.Slot0.kP = kP;
    pivotConfig.Slot0.kI = kI;
    pivotConfig.Slot0.kD = kD;
    pivotConfig.Slot0.kG = kG;
    pivotConfig.Slot0.kS = kS;
    PhoenixUtil.tryUntilOk(5, () -> pivotTalonFX.getConfigurator().apply(pivotConfig, 0.050));
  }

  @Override
  public void setPivotZero() {
    pivotTalonFX.setPosition(0.0);
  }
}
