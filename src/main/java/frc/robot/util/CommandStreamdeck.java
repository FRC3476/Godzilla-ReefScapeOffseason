package frc.robot.util;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandStreamdeck {

    NetworkTable table;

    public CommandStreamdeck(String NetworkTableName) {
        this.table = NetworkTableInstance.getDefault().getTable(NetworkTableName);
    }

    private NetworkTableEntry getButtonEntry(int buttonId) {
        return table.getEntry(Integer.toString(buttonId));
    }

    private NetworkTableEntry getImageEntry(int buttonId) {
        return table.getEntry(Integer.toString(buttonId)+"image");
    }

    public Trigger button(int buttonId) {
        return new Trigger(() -> getButtonEntry(buttonId).getBoolean(false));
    }

    public Command setButtonImage(int buttonId, Boolean appearsOn) {
        return new InstantCommand(() -> getImageEntry(buttonId).setBoolean(appearsOn));
    }
    
}
