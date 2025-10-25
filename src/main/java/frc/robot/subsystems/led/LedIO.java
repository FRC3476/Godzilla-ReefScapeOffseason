package frc.robot.subsystems.led;

public interface LedIO {
  class LedInputs {}

  default void readInputs(LedIO.LedInputs inputs) {}

  default void update(final LedIO.LedInputs inputs) {}

  default LedState getCurrentState() {
    return LedState.kOff;
  }
  ;

  default void writePixels(LedState state) {}
  ;

  default void writePixels(LedState[] states) {}
  ;

  default void blink(LedState state, double duration) {}
  ;

  default void fire() {}
  ;

  default void rainbow() {}
  ;

  public default void percentageFull(double percent, LedState state) {}
  ;
}
