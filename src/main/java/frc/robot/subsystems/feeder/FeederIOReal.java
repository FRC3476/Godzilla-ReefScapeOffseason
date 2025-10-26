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
import frc.robot.Constants;
import frc.robot.Constants.FeederConstants;
import frc.robot.util.CANDiagnostics;
import frc.robot.util.MotorStallDetection;
import frc.robot.util.PhoenixUtil;

public class FeederIOReal implements FeederIO {
  private boolean directionReversed = false;

  protected final TalonFX rightRoller;
  protected final TalonFX leftRoller;

  private final CANrange canRange;
  private final CANrange frontCanRange;

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

  private final StatusSignal<Boolean> frontCanRangeTripped;
  private final StatusSignal<Double> frontCanRangeSignalStrength;
  private final StatusSignal<Distance> frontCanRangeDistance;

  private final BaseStatusSignal[] signals;

  public FeederIOReal() {
    // Initialize hardware
    rightRoller = new TalonFX(FeederConstants.RIGHT_ID, Constants.MISC_CANIVORE);
    leftRoller = new TalonFX(FeederConstants.LEFT_ID, Constants.MISC_CANIVORE);
    canRange = new CANrange(FeederConstants.CANRANGE_ID, Constants.MISC_CANIVORE);
    frontCanRange = new CANrange(FeederConstants.FRONT_CANRANGE_ID, Constants.MISC_CANIVORE);

    // Apply configs
    PhoenixUtil.tryUntilOk(
        5, () -> rightRoller.getConfigurator().apply(FeederConstants.ROLLER_TALON_CONFIG));
    PhoenixUtil.tryUntilOk(
        5, () -> canRange.getConfigurator().apply(FeederConstants.CANRANGE_CONFIG));
    PhoenixUtil.tryUntilOk(
        5, () -> frontCanRange.getConfigurator().apply(FeederConstants.CANRANGE_CONFIG));

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

    frontCanRangeTripped = frontCanRange.getIsDetected();
    frontCanRangeSignalStrength = frontCanRange.getSignalStrength();
    frontCanRangeDistance = frontCanRange.getDistance();

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
        canRangeDistance,
        frontCanRangeTripped,
        frontCanRangeSignalStrength,
        frontCanRangeDistance);

    rightRoller.optimizeBusUtilization();
    leftRoller.optimizeBusUtilization();
    canRange.optimizeBusUtilization();
    frontCanRange.optimizeBusUtilization();

    signals =
        new BaseStatusSignal[] {
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
          canRangeDistance,
          frontCanRangeTripped,
          frontCanRangeSignalStrength,
          frontCanRangeDistance
        };
  }

  public void updateInputs(FeederIOInputs inputs) {
    BaseStatusSignal.refreshAll(signals);
    CANDiagnostics.checkSignalHealth("Feeder", signals);

    inputs.rightRollerData =
        new F_RollerData(
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
        new F_RollerData(
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
        new F_CanRangeData(
            BaseStatusSignal.isAllGood(canRangeTripped, canRangeDistance, canRangeSignalStrength),
            canRangeTripped.getValue(),
            canRangeSignalStrength.getValueAsDouble(),
            canRangeDistance.getValueAsDouble());

    inputs.frontCanRangeData =
        new FRONT_CanRangeData(
            BaseStatusSignal.isAllGood(
                frontCanRangeTripped, frontCanRangeDistance, frontCanRangeSignalStrength),
            frontCanRangeTripped.getValue(),
            frontCanRangeSignalStrength.getValueAsDouble(),
            frontCanRangeDistance.getValueAsDouble());
  }

  @Override
  public void setRollerVoltage(double voltage) {
    if (directionReversed) {
      leftRoller.setControl(new Follower(FeederConstants.RIGHT_ID, true));
      directionReversed = false;
    }
    rightRoller.setControl(new VoltageOut(voltage));
  }

  @Override
  public void setRollerVoltageReversed(double voltage) {
    if (!directionReversed) {
      leftRoller.setControl(new Follower(FeederConstants.RIGHT_ID, false));
      directionReversed = true;
    }
    rightRoller.setControl(new VoltageOut(voltage));
  }

  @Override
  public boolean checkMotorsStalled() {
    return MotorStallDetection.isMotorStalled(
            rightRollerStatorCurrent.getValueAsDouble(),
            rightRollerVelocityRPS.getValueAsDouble(),
            FeederConstants.STALLED_CURRENT,
            FeederConstants.STALLED_RPS)
        || MotorStallDetection.isMotorStalled(
            leftRollerStatorCurrent.getValueAsDouble(),
            leftRollerVelocityRPS.getValueAsDouble(),
            FeederConstants.STALLED_CURRENT,
            FeederConstants.STALLED_RPS);
  }
}
