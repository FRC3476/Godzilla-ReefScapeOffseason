package frc.robot.util.Controls;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.stream.IntStream;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class StreamDeck extends SubsystemBase {
  private final Map<StreamDeckButton, ButtonRecord> buttonMap = new HashMap<>();

  private static record ButtonRecord(
      LoggedNetworkBoolean pressed,
      LoggedNetworkBoolean pressedPrev,
      LoggedNetworkBoolean toggled,
      BooleanSupplier selected,
      BooleanPublisher activePub,
      ButtonType type) {}

  @Override
  public void periodic() {
    buttonMap
        .values()
        .forEach(
            button -> {
              if (button.type == ButtonType.TOGGLE
                  && button.pressed.get()
                  && !button.pressedPrev.get()) {
                button.toggled.set(!button.toggled.get());
              }
              button.activePub.set(button.selected.getAsBoolean());
              button.pressedPrev.set(button.pressed.get());
            });
  }

  public void configureDefaultButtons(Set<StreamDeckButton> buttons) {
    configureButton(
        config -> {
          for (StreamDeckButton button : buttons) {
            config.addDefault(button);
          }
        });
  }

  public void configureToggleButtons(Set<StreamDeckButton> buttons) {
    configureButton(
        config -> {
          for (StreamDeckButton button : buttons) {
            config.addToggle(button);
          }
        });
  }

  public void configureCustomButtons(Map<StreamDeckButton, BooleanSupplier> buttonMap) {
    configureButton(
        config -> {
          for (Map.Entry<StreamDeckButton, BooleanSupplier> entry : buttonMap.entrySet()) {
            config.add(entry.getKey(), entry.getValue());
          }
        });
  }

  public void clearButtons() {
    for (Map.Entry<StreamDeckButton, ButtonRecord> entry : buttonMap.entrySet()) {
      if (entry.getValue().type  != ButtonType.PAGE_SELECTOR) {
        buttonMap.remove(entry.getKey());
      }
    }
  }

  public void clearAllButtons() {
    for (Map.Entry<StreamDeckButton, ButtonRecord> entry : buttonMap.entrySet()) {
      buttonMap.remove(entry.getKey());
    }
  }

  private StreamDeck configureButton(Consumer<ButtonConfiguration> config) {
    var configuration = new ButtonConfiguration();
    config.accept(configuration);

    var nt = NetworkTableInstance.getDefault();
    var deckTable = nt.getTable("StreamDeck");
    List<String> networkTableKeys = StreamDeckButton.getNetworkTableKeys();
    configuration.buttonConfigurations.forEach(
        (button, configInfo) -> {
          Optional<BooleanSupplier> selected = configInfo.getActiveSupplier();
          ButtonType type = configInfo.getType();

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
          var loggedBooleanPrev = new LoggedNetworkBoolean(dataToPublish.get(0) + "Prev", false);
          var loggedBooleanToggled =
              new LoggedNetworkBoolean(dataToPublish.get(0) + "Toggled", false);
          buttonMap.put(
              button,
              new ButtonRecord(
                  loggedBoolean,
                  loggedBooleanPrev,
                  loggedBooleanToggled,
                  selected.orElse(
                      type == ButtonType.TOGGLE
                          ? () -> loggedBooleanToggled.get()
                          : loggedBoolean::get),
                  table.getBooleanTopic("Selected").publish(),
                  type));
        });

    deckTable.getIntegerTopic("LastModified").publish().set(Logger.getTimestamp());

    return this;
  }

  public Trigger button(StreamDeckButton button) {
    if (!buttonMap.containsKey(button)) {
      StreamDeckAlert.warning(
              "Stream Deck button trigger added for invalid button " + button.getIndex())
          .enable();
      return new Trigger(() -> false);
    }

    return new Trigger(buttonMap.get(button).pressed::get);
  }

  // public ButtonGroup buttonGroup() {
  //   return new ButtonGroup();
  // }

  public enum ButtonType {
    PRESS,
    TOGGLE,
    CUSTOM,
    PAGE_SELECTOR
  }

  public class ButtonConfiguration {
    private class ButtonConfigurationInfo {
      private Optional<BooleanSupplier> activeSupplier;
      private ButtonType type;

      public ButtonConfigurationInfo(Optional<BooleanSupplier> activeSupplier,
      ButtonType type) {
        this.activeSupplier = activeSupplier;
        this.type = type;
      }
      public ButtonConfigurationInfo(ButtonType type) {
        this.activeSupplier = Optional.empty();
        this.type = type;
      }

      public Optional<BooleanSupplier> getActiveSupplier() {
        return activeSupplier;
      }
      public ButtonType getType() {
        return type;
      }

    }
    private final Map<StreamDeckButton, ButtonConfigurationInfo>
        buttonConfigurations = new HashMap<>();

    private ButtonConfiguration() {}

    public ButtonConfiguration addDefault(StreamDeckButton button) {
      buttonConfigurations.put(button, new ButtonConfigurationInfo(ButtonType.PRESS));
      return this;
    }
    

    public ButtonConfiguration addToggle(StreamDeckButton button) {
      buttonConfigurations.put(button, new ButtonConfigurationInfo(ButtonType.TOGGLE));
      return this;
    }

    public ButtonConfiguration add(StreamDeckButton button, BooleanSupplier selected) {
      buttonConfigurations.put(button, new ButtonConfigurationInfo(Optional.of(selected), ButtonType.CUSTOM));
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
