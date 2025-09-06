package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.FeederConstants;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.MotorStallDetection;

public class FeederIOReal implements FeederIO {
  private final TalonFX rightRoller;
  private final TalonFX leftRoller;

  private final CANrange canRange;

  // Right roller status signals
  private final StatusSignal<Voltage> rightRollerVoltage;
  private final StatusSignal<Current> rightRollerSupplyCurrent;
  private final StatusSignal<Current> rightRollerStatorCurrent;
  private final StatusSignal<Temperature> rightRollerTemperature;
  private final StatusSignal<AngularVelocity> rightRollerVelocityRPS;

  // Left roller status signals
  private final StatusSignal<Voltage> leftRollerVoltage;
  private final StatusSignal<Current> leftRollerSupplyCurrent;
  private final StatusSignal<Current> leftRollerStatorCurrent;
  private final StatusSignal<Temperature> leftRollerTemperature;
  private final StatusSignal<AngularVelocity> leftRollerVelocityRPS;

  // CANRange status signals
  private final StatusSignal<Boolean> canRangeTripped;
  private final StatusSignal<Double> canRangeSignalStrength;
  private final StatusSignal<Distance> canRangeDistance;

  public FeederIOReal() {
    // Initialize hardware
    rightRoller = new TalonFX(FeederConstants.RIGHT_ID);
    leftRoller = new TalonFX(FeederConstants.LEFT_ID);
    canRange = new CANrange(FeederConstants.CANRANGE_ID);

    // Apply configs
    rightRoller.getConfigurator().apply(FeederConstants.ROLLER_TALON_CONFIG);
    canRange.getConfigurator().apply(FeederConstants.CANRANGE_CONFIG);

    // Set up left roller to follow right roller
    leftRoller.setControl(new Follower(FeederConstants.RIGHT_ID, true));

    // Initialize status signals
    rightRollerVoltage = rightRoller.getMotorVoltage();
    rightRollerSupplyCurrent = rightRoller.getSupplyCurrent();
    rightRollerStatorCurrent = rightRoller.getStatorCurrent();
    rightRollerTemperature = rightRoller.getDeviceTemp();
    rightRollerVelocityRPS = rightRoller.getRotorVelocity();

    leftRollerVoltage = leftRoller.getMotorVoltage();
    leftRollerSupplyCurrent = leftRoller.getSupplyCurrent();
    leftRollerStatorCurrent = leftRoller.getStatorCurrent();
    leftRollerTemperature = leftRoller.getDeviceTemp();
    leftRollerVelocityRPS = leftRoller.getRotorVelocity();

    canRangeTripped = canRange.getIsDetected();
    canRangeSignalStrength = canRange.getSignalStrength();
    canRangeDistance = canRange.getDistance();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        rightRollerVoltage,
        rightRollerSupplyCurrent,
        rightRollerStatorCurrent,
        rightRollerTemperature,
        rightRollerVelocityRPS,
        leftRollerVoltage,
        leftRollerSupplyCurrent,
        leftRollerStatorCurrent,
        leftRollerTemperature,
        leftRollerVelocityRPS,
        canRangeTripped,
        canRangeSignalStrength,
        canRangeDistance);

    rightRoller.optimizeBusUtilization();
    leftRoller.optimizeBusUtilization();
    canRange.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        false,
        rightRollerVoltage,
        rightRollerSupplyCurrent,
        rightRollerStatorCurrent,
        rightRollerTemperature,
        rightRollerVelocityRPS,
        leftRollerVoltage,
        leftRollerSupplyCurrent,
        leftRollerStatorCurrent,
        leftRollerTemperature,
        leftRollerVelocityRPS,
        canRangeTripped,
        canRangeSignalStrength,
        canRangeDistance);
  }

  public void updateInputs(FeederIOInputs inputs) {
    inputs.rightRollerData =
        new RollerData(
            BaseStatusSignal.isAllGood(
                rightRollerVoltage,
                rightRollerSupplyCurrent,
                rightRollerStatorCurrent,
                rightRollerTemperature,
                rightRollerVelocityRPS),
            rightRollerVoltage.getValueAsDouble(),
            rightRollerSupplyCurrent.getValueAsDouble(),
            rightRollerStatorCurrent.getValueAsDouble(),
            rightRollerTemperature.getValueAsDouble(),
            rightRollerVelocityRPS.getValueAsDouble());

    inputs.leftRollerData =
        new RollerData(
            BaseStatusSignal.isAllGood(
                leftRollerVoltage,
                leftRollerSupplyCurrent,
                leftRollerStatorCurrent,
                leftRollerTemperature,
                leftRollerVelocityRPS),
            leftRollerVoltage.getValueAsDouble(),
            leftRollerSupplyCurrent.getValueAsDouble(),
            leftRollerStatorCurrent.getValueAsDouble(),
            leftRollerTemperature.getValueAsDouble(),
            leftRollerVelocityRPS.getValueAsDouble());

    inputs.canRangeData =
        new CanRangeData(
            BaseStatusSignal.isAllGood(canRangeTripped, canRangeDistance, canRangeSignalStrength),
            canRangeTripped.getValue(),
            canRangeSignalStrength.getValueAsDouble(),
            canRangeDistance.getValueAsDouble());
  }

  @Override
  public void setRollerVoltage(double voltage) {
    rightRoller.setControl(new VoltageOut(0.0).withOutput(voltage));
  }

  @Override
  public void checkForJam() {
    if (MotorStallDetection.isMotorStalled(rightRoller, 10.0, 10.0)) {

    }
  }
}
