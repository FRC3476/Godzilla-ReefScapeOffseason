package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Led extends SubsystemBase {
  private final LedIO io;
  private final RobotState state;

  private static final LoggedTunableNumber orangeR =
      new LoggedTunableNumber("LED/Orange R", LedState.kCOOrangeLed.red);
  private static final LoggedTunableNumber orangeG =
      new LoggedTunableNumber("LED/Orange G", LedState.kCOOrangeLed.green); // Placeholder value
  private static final LoggedTunableNumber orangeB =
      new LoggedTunableNumber("LED/Orange B", LedState.kCOOrangeLed.blue);
  private static final LoggedTunableNumber tealR =
      new LoggedTunableNumber("LED/Teal R", LedState.kCOTealLed.red);
  private static final LoggedTunableNumber tealG =
      new LoggedTunableNumber("LED/Teal G", LedState.kCOTealLed.green); // Placeholder value
  private static final LoggedTunableNumber tealB =
      new LoggedTunableNumber("LED/Teal B", LedState.kCOTealLed.blue);

  public record PercentageSetpoint(double pct, LedState color) {}

  public Led(final LedIO io, RobotState state) {
    this.io = io;
    this.state = state;
  }

  @Override
  public void periodic() {
    super.periodic();

    RobotState.setLedState(getCurrentState());
    Logger.recordOutput(
        "LED/currentCommand",
        (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());
  }

  public LedState getCurrentState() {
    return io.getCurrentState();
  }

  /* change runOnce to run in case we have to keep setting the LED color periodically? */
  public Command commandSolidColor(LedState state) {
    return run(() -> setSolidColor(state)).ignoringDisable(true).withName("LED Solid Color");
  }

  public Command commandSolidColor(Supplier<LedState> state) {
    return run(() -> setSolidColor(state.get())).ignoringDisable(true).withName("LED Solid Color");
  }

  public Command commandOff() {
    return commandSolidColor(LedState.kOff);
  }

  public Command commandSetTeal() {
    return commandSolidColor(LedState.kCOTealLed);
  }

  public Command commandSetOrange() {
    return commandSolidColor(LedState.kCOOrangeLed);
  }

  public Command commandSolidPattern(LedState[] states) {
    return run(() -> setSolidPattern(states)).ignoringDisable(true).withName("LED Solid Pattern");
  }

  public Command commandPercentageFull(DoubleSupplier percentageFull, LedState state) {
    return run(() -> setPercentageFull(percentageFull.getAsDouble(), state)).ignoringDisable(true);
  }

  public Command commandPercentageFull(Supplier<PercentageSetpoint> percentageSupplier) {
    return run(() ->
            setPercentageFull(percentageSupplier.get().pct, percentageSupplier.get().color))
        .ignoringDisable(true);
  }

  public Command commandBlinkingState(LedState state, double duration) {
    return this.runOnce(() -> this.io.blink(state, duration));
  }

  public Command commandFire() {
    return this.runOnce(() -> this.io.fire()).ignoringDisable(true).withName("LED Fire");
  }

  public Command commandRainbow() {
    return this.runOnce(() -> this.io.rainbow()).ignoringDisable(true).withName("LED Rainbow");
  }

  private void setSolidColor(LedState state) {
    io.writePixels(state);
  }

  private void setSolidPattern(LedState[] states) {
    io.writePixels(states);
  }

  // private void setPercentageFull(double percentageFull, LedState state) {
  //   LedState[] pixels = new LedState[Constants.LedConstants.kMaxLEDCount / 2];
  //   for (int i = 0; i < pixels.length; i++) {
  //     if (i < pixels.length * MathUtil.clamp(percentageFull, 0.0, 1.0)) {
  //       pixels[i] = state;
  //     }
  //   }
  // }

  private void setPercentageFull(double percent, LedState state) {
    this.io.percentageFull(percent, state);
  }

  @SuppressWarnings("unused")
  private LedState[] mirror(LedState[] pixels) {
    LedState[] fullPixels = new LedState[Constants.LedConstants.kMaxLEDCount];

    for (int i = Constants.LedConstants.kCandleLEDCount;
        i
            < Constants.LedConstants.kCandleLEDCount
                + (Constants.LedConstants.kNonCandleLEDCount / 2);
        i++) {
      fullPixels[fullPixels.length - i - 2] = pixels[i];
      fullPixels[i] = pixels[i];
    }

    return fullPixels;
  }
}
