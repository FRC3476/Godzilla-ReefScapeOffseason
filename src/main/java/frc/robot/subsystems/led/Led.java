package frc.robot.subsystems.led;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Led extends SubsystemBase {
  private final LedIO io;
  // private final RobotState state;

  public record PercentageSetpoint(double pct, LedState color) {}

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

  public Led(final LedIO io) { // RobotState state
    this.io = io;
    // this.state = state;
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
    return this.runOnce(() -> setSolidColor(state))
        .ignoringDisable(true)
        .withName("LED Solid Color");
  }

  public Command commandSolidColor(Supplier<LedState> state) {
    return this.run(() -> setSolidColor(state.get()))
        .ignoringDisable(true)
        .withName("LED Solid Color");
  }

  public Command commandSolidPattern(LedState[] states) {
    return this.runOnce(() -> setSolidPattern(states))
        .ignoringDisable(true)
        .withName("LED Solid Pattern");
  }

  public Command commandPercentageFull(DoubleSupplier percentageFull, LedState state) {
    return this.run(() -> setPercentageFull(percentageFull.getAsDouble(), state))
        .ignoringDisable(true);
  }

  public Command commandPercentageFull(Supplier<PercentageSetpoint> percentageSupplier) {
    return this.run(
            () -> setPercentageFull(percentageSupplier.get().pct, percentageSupplier.get().color))
        .ignoringDisable(true);
  }

  public Command commandBlinkingState(
      LedState stateOne, LedState stateTwo, double durationOne, double durationTwo) {
    return this.runOnce(() -> setSolidColor(stateOne))
        .andThen(new WaitCommand(durationOne))
        .andThen(this.runOnce(() -> setSolidColor(stateTwo)))
        .andThen(new WaitCommand(durationTwo))
        .repeatedly()
        .ignoringDisable(true)
        .withName("Blinking LED command");
  }

  public Command commandBlinkingStateWithoutScheduler(
      LedState stateOne, LedState stateTwo, double durationOne, double durationTwo) {
    var state =
        new Object() {
          public boolean color1 = true;
          public double timestamp = Timer.getFPGATimestamp();
        };
    return Commands.runOnce(
            () -> {
              state.color1 = true;
              state.timestamp = Timer.getFPGATimestamp();
            })
        .andThen(
            commandSolidColor(
                () -> {
                  if (state.color1 && state.timestamp + durationOne <= Timer.getFPGATimestamp()) {
                    state.color1 = false;
                    state.timestamp = Timer.getFPGATimestamp();
                  } else if (!state.color1
                      && state.timestamp + durationTwo <= Timer.getFPGATimestamp()) {
                    state.color1 = true;
                    state.timestamp = Timer.getFPGATimestamp();
                  }

                  if (state.color1) {
                    return stateOne;
                  } else {
                    return stateTwo;
                  }
                }))
        .ignoringDisable(true)
        .withName("Blinking LED command");
  }

  public Command commandBlinkingState(LedState stateOne, LedState stateTwo, double duration) {
    return commandBlinkingState(stateOne, stateTwo, duration, duration).ignoringDisable(true);
  }

  public Command commandFire() {
    return this.runOnce(() -> this.io.fire()).ignoringDisable(true).withName("LED Fire");
  }

  public Command commandRainbow() {
    return this.runOnce(() -> this.io.rainbow()).ignoringDisable(true).withName("LED Rainbow");
  }

  public Command commandSetTeal() {
    return this.runOnce(
            () ->
                setSolidColor(
                    new LedState((int) tealR.get(), (int) tealG.get(), (int) tealB.get())))
        .ignoringDisable(true)
        .withName("Set LED Teal");
  }

  public Command commandSetOrange() {
    return this.runOnce(
            () ->
                setSolidColor(
                    new LedState((int) orangeR.get(), (int) orangeG.get(), (int) orangeB.get())))
        .ignoringDisable(true)
        .withName("Set LED Orange");
  }

  private void setSolidColor(LedState state) {
    io.writePixels(state);
  }

  private void setSolidPattern(LedState[] states) {
    io.writePixels(states);
  }

  private void setPercentageFull(double percentageFull, LedState state) {
    LedState[] pixels = new LedState[Constants.LEDConstants.kMaxLEDCount / 2];
    for (int i = 0; i < pixels.length; i++) {
      if (i < pixels.length * MathUtil.clamp(percentageFull, 0.0, 1.0)) {
        pixels[i] = state;
      }
    }
  }

  @SuppressWarnings("unused")
  private LedState[] mirror(LedState[] pixels) {
    LedState[] fullPixels = new LedState[Constants.LEDConstants.kMaxLEDCount];

    for (int i = Constants.LEDConstants.kCandleLEDCount;
        i
            < Constants.LEDConstants.kCandleLEDCount
                + (Constants.LEDConstants.kNonCandleLEDCount / 2);
        i++) {
      fullPixels[fullPixels.length - i - 2] = pixels[i];
      fullPixels[i] = pixels[i];
    }

    return fullPixels;
  }
}
