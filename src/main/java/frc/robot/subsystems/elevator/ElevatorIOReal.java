package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.util.MotorStallDetection;
import frc.robot.util.PhoenixUtil;

public class ElevatorIOReal implements ElevatorIO {

  protected TalonFX rightTalon;
  protected TalonFX leftTalon;
  protected TalonFX extraTalon;

  private MotionMagicVoltage m_request =
      new MotionMagicVoltage(PhysicalConstants.ABSOLUTE_ZERO).withEnableFOC(true);
  private VoltageOut m_VoltageOut =
      new VoltageOut(PhysicalConstants.ABSOLUTE_ZERO).withEnableFOC(true);

  // =====Logged Values=====
  StatusSignal<Angle> rightPosition;
  StatusSignal<Voltage> rightAppliedVolts;
  StatusSignal<Current> rightTorqueCurrentAmps;
  StatusSignal<Current> rightSupplyCurrentAmps;
  StatusSignal<Temperature> rightTempCelsius;

  StatusSignal<Angle> leftPosition;
  StatusSignal<Voltage> leftAppliedVolts;
  StatusSignal<Current> leftTorqueCurrentAmps;
  StatusSignal<Current> leftSupplyCurrentAmps;
  StatusSignal<Temperature> leftTempCelsius;

  StatusSignal<Angle> extraPosition;
  StatusSignal<Voltage> extraAppliedVolts;
  StatusSignal<Current> extraTorqueCurrentAmps;
  StatusSignal<Current> extraSupplyCurrentAmps;
  StatusSignal<Temperature> extraTempCelsius;

  private final BaseStatusSignal[] signals;

  public ElevatorIOReal() {
    rightTalon = new TalonFX(ElevatorConstants.elevatorRightID, Constants.misc_canivore);
    leftTalon = new TalonFX(ElevatorConstants.elevatorLeftID, Constants.misc_canivore);
    extraTalon = new TalonFX(ElevatorConstants.elevatorExtraID, Constants.misc_canivore);

    PhoenixUtil.tryUntilOk(
        5, () -> rightTalon.getConfigurator().apply(ElevatorConstants.elevatorRightTalon));
    leftTalon.setControl(new Follower(ElevatorConstants.elevatorRightID, true));
    extraTalon.setControl(new Follower(ElevatorConstants.elevatorRightID, true));

    rightPosition = rightTalon.getPosition();
    rightAppliedVolts = rightTalon.getMotorVoltage();
    rightTorqueCurrentAmps = rightTalon.getTorqueCurrent();
    rightSupplyCurrentAmps = rightTalon.getSupplyCurrent();
    rightTempCelsius = rightTalon.getDeviceTemp();

    leftPosition = leftTalon.getPosition();
    leftAppliedVolts = leftTalon.getMotorVoltage();
    leftTorqueCurrentAmps = leftTalon.getTorqueCurrent();
    leftSupplyCurrentAmps = leftTalon.getSupplyCurrent();
    leftTempCelsius = leftTalon.getDeviceTemp();

    extraPosition = extraTalon.getPosition();
    extraAppliedVolts = extraTalon.getMotorVoltage();
    extraTorqueCurrentAmps = extraTalon.getTorqueCurrent();
    extraSupplyCurrentAmps = extraTalon.getSupplyCurrent();
    extraTempCelsius = extraTalon.getDeviceTemp();

    signals =
        new BaseStatusSignal[] {
          rightPosition,
          rightAppliedVolts,
          rightTorqueCurrentAmps,
          rightSupplyCurrentAmps,
          rightTempCelsius,
          leftPosition,
          leftAppliedVolts,
          leftTorqueCurrentAmps,
          leftSupplyCurrentAmps,
          leftTempCelsius,
          extraPosition,
          extraAppliedVolts,
          extraTorqueCurrentAmps,
          extraSupplyCurrentAmps,
          extraTempCelsius
        };

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        rightPosition,
        rightAppliedVolts,
        rightTorqueCurrentAmps,
        rightSupplyCurrentAmps,
        rightTempCelsius,
        leftPosition,
        leftAppliedVolts,
        leftTorqueCurrentAmps,
        leftSupplyCurrentAmps,
        leftTempCelsius,
        extraPosition,
        extraAppliedVolts,
        extraTorqueCurrentAmps,
        extraSupplyCurrentAmps,
        extraTempCelsius);
    ParentDevice.optimizeBusUtilizationForAll(rightTalon, leftTalon, extraTalon);
    PhoenixUtil.registerSignals(
        false,
        rightPosition,
        rightAppliedVolts,
        rightTorqueCurrentAmps,
        rightSupplyCurrentAmps,
        rightTempCelsius,
        leftPosition,
        leftAppliedVolts,
        leftTorqueCurrentAmps,
        leftSupplyCurrentAmps,
        leftTempCelsius,
        extraPosition,
        extraAppliedVolts,
        extraTorqueCurrentAmps,
        extraSupplyCurrentAmps,
        extraTempCelsius);
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(signals);

    inputs.data =
        new ElevatorIOData(
            BaseStatusSignal.isAllGood(
                rightPosition,
                rightAppliedVolts,
                rightTorqueCurrentAmps,
                rightSupplyCurrentAmps,
                rightTempCelsius),
            BaseStatusSignal.isAllGood(
                leftPosition,
                leftAppliedVolts,
                leftTorqueCurrentAmps,
                leftSupplyCurrentAmps,
                leftTempCelsius),
            BaseStatusSignal.isAllGood(
                extraPosition,
                extraAppliedVolts,
                extraTorqueCurrentAmps,
                extraSupplyCurrentAmps,
                extraTempCelsius),
            Units.rotationsToRadians(rightPosition.getValueAsDouble()),
            rightAppliedVolts.getValueAsDouble(),
            rightTorqueCurrentAmps.getValueAsDouble(),
            rightSupplyCurrentAmps.getValueAsDouble(),
            rightTempCelsius.getValueAsDouble(),
            Units.rotationsToRadians(leftPosition.getValueAsDouble()),
            leftAppliedVolts.getValueAsDouble(),
            leftTorqueCurrentAmps.getValueAsDouble(),
            leftSupplyCurrentAmps.getValueAsDouble(),
            leftTempCelsius.getValueAsDouble(),
            Units.rotationsToRadians(extraPosition.getValueAsDouble()),
            extraAppliedVolts.getValueAsDouble(),
            extraTorqueCurrentAmps.getValueAsDouble(),
            extraSupplyCurrentAmps.getValueAsDouble(),
            extraTempCelsius.getValueAsDouble());
  }

  @Override
  public void setElevatorVoltage(double voltage) {
    rightTalon.setControl(m_VoltageOut.withOutput(voltage));
  }

  @Override
  public void setElevatorTargetPosition(double position) {
    rightTalon.setControl(m_request.withPosition(position));
  }

  @Override
  public void setElevatorZero() {
    rightTalon.setPosition(0.0);
    leftTalon.setPosition(0.0);
    extraTalon.setPosition(0.0);
  }

  public void stop() {
    rightTalon.stopMotor();
  }

  @Override
  public boolean checkMotorsStalled() {
    return MotorStallDetection.isMotorStalled(
            rightTalon, ElevatorConstants.STALLED_CURRENT, ElevatorConstants.STALLED_RPS)
        || MotorStallDetection.isMotorStalled(
            leftTalon, ElevatorConstants.STALLED_CURRENT, ElevatorConstants.STALLED_RPS);
  }
}
