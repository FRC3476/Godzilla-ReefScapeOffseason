package frc.robot.subsystems.led;

public interface LedIO {
    class LedInputs {}

    default void readInputs(LedIO.LedInputs inputs) {}

    default void update(final LedIO.LedInputs inputs) {}

    LedState getCurrentState();

    void writePixels(LedState state);

    void writePixels(LedState[] states);
}
