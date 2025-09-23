package frc.robot.subsystems.led;

public class LedIOSim implements LedIO {
  private LedState currentState = LedState.kOff;

  @Override
  public LedState getCurrentState() {
    return currentState;
  }

  @Override
  public void writePixels(LedState state) {
    if (state != null) {
      currentState = state;
    }
  }

  @Override
  public void writePixels(LedState[] states) {
    if (states != null && states.length > 0 && states[0] != null) {
      currentState = states[0];
    }
  }
}