package frc.robot.util.Controls;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.Alert;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.stream.IntStream;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class StreamDeck extends SubsystemBase {
  private final Map<StreamDeckButton, ButtonRecord> buttonMap = new HashMap<>();

  private static record ButtonRecord(
      LoggedNetworkBoolean pressed, BooleanSupplier selected, BooleanPublisher activePub) {}

  @Override
  public void periodic() {
    buttonMap.values().forEach(button -> button.activePub.set(button.selected.getAsBoolean()));
  }

  public StreamDeck configureButton(Consumer<ButtonConfiguration> config) {
    var configuration = new ButtonConfiguration();
    config.accept(configuration);

    var nt = NetworkTableInstance.getDefault();
    var deckTable = nt.getTable("StreamDeck");
    List<String> networkTableKeys = StreamDeckButton.getNetworkTableKeys();
    configuration.buttonConfigurations.forEach(
        (button, selected) -> {
          var table = deckTable.getSubTable("Button/" + button.getIndex());
          List<String> dataToPublish = button.getDataToPublish();
          IntStream.range(0, Math.min(networkTableKeys.size(), dataToPublish.size()))
              .forEach(
                  i ->
                      table
                          .getStringTopic(networkTableKeys.get(i))
                          .publish()
                          .set(dataToPublish.get(i)));

          var loggedBoolean = new LoggedNetworkBoolean(dataToPublish.get(0), false);
          buttonMap.put(
              button,
              new ButtonRecord(
                  loggedBoolean,
                  selected.orElse(loggedBoolean::get),
                  table.getBooleanTopic("Selected").publish()));
        });

    deckTable.getIntegerTopic("LastModified").publish().set(Logger.getTimestamp());

    return this;
  }

  public Trigger button(StreamDeckButton button) {
    if (!buttonMap.containsKey(button)) {
      Alert.warning("Stream Deck button trigger added for invalid button " + button.getIndex())
          .enable();
      return new Trigger(() -> false);
    }

    return new Trigger(buttonMap.get(button).pressed::get);
  }

  public ButtonGroup buttonGroup() {
    return new ButtonGroup();
  }

  public class ButtonConfiguration {
    private final Map<StreamDeckButton, Optional<BooleanSupplier>> buttonConfigurations =
        new HashMap<>();

    private ButtonConfiguration() {}

    public ButtonConfiguration addDefault(StreamDeckButton button) {
      buttonConfigurations.put(button, Optional.empty());
      return this;
    }

    public ButtonConfiguration add(StreamDeckButton button, BooleanSupplier selected) {
      buttonConfigurations.put(button, Optional.of(selected));
      return this;
    }
  }

  public class ButtonGroup {
    private final Map<StreamDeckButton, Trigger> triggers = new HashMap<>();
    private Optional<StreamDeckButton> selected = Optional.empty();

    private ButtonGroup() {}

    public ButtonGroup option(StreamDeckButton button) {
      initButton(button);
      return this;
    }

    public ButtonGroup option(StreamDeckButton button, Consumer<Trigger> trigger) {
      var buttonTrigger = initButton(button);
      trigger.accept(buttonTrigger);
      return this;
    }

    public ButtonGroup select(StreamDeckButton button) {
      selected = Optional.of(button);
      return this;
    }

    public ButtonGroup clear() {
      selected = Optional.empty();
      return this;
    }

    public Optional<StreamDeckButton> getSelected() {
      return selected;
    }

    public boolean isSelected(StreamDeckButton button) {
      return selected.isPresent() && selected.get() == button;
    }

    public Trigger trigger(StreamDeckButton button) {
      if (triggers.containsKey(button)) {
        return triggers.get(button);
      }

      return initButton(button);
    }

    private Trigger initButton(StreamDeckButton button) {
      button(button)
          .onTrue(
              Commands.runOnce(() -> selected = Optional.of(button))
                  .ignoringDisable(true)
                  .withName("UpdateStreamDeckButtonGroupActiveIndex"));

      var trigger = new Trigger(() -> isSelected(button));
      triggers.put(button, trigger);
      return trigger;
    }
  }
}
