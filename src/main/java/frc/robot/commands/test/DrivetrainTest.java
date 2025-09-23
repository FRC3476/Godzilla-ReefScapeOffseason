package frc.robot.commands.test;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Drive;

public class DrivetrainTest extends SequentialCommandGroup {
  /** Creates a new DrivetrainTest. */
  public DrivetrainTest(Drive drive) {
    addCommands(
        drive.run(() -> drive.runCharacterization(0.2)),
        new WaitCommand(1.0),
        drive.run(drive::stop),
        new WaitCommand(0.5),
        drive.run(() -> drive.runCharacterization(-0.2)),
        new WaitCommand(1.0),
        drive.run(drive::stop),
        new WaitCommand(0.5),

        // Forward/backward movement
        drive.run(() -> drive.runVelocity(new ChassisSpeeds(0.5, 0.0, 0.0))),
        new WaitCommand(2.0),
        drive.run(drive::stop),
        new WaitCommand(0.5),
        drive.run(() -> drive.runVelocity(new ChassisSpeeds(-0.5, 0.0, 0.0))),
        new WaitCommand(2.0),
        drive.run(drive::stop),
        new WaitCommand(0.5),

        //  Lateral movement
        drive.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, 0.5, 0.0))),
        new WaitCommand(2.0),
        drive.run(drive::stop),
        new WaitCommand(0.5),
        drive.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, -0.5, 0.0))),
        new WaitCommand(2.0),
        drive.run(drive::stop),
        new WaitCommand(0.5),

        // Test 4: Rotational movement
        drive.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.5))),
        new WaitCommand(2.0),
        drive.run(drive::stop),
        new WaitCommand(0.5),
        drive.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, 0.0, -0.5))),
        new WaitCommand(2.0),
        drive.run(drive::stop),
        new WaitCommand(0.5),
        
        // X-lock test 
        drive.run(drive::stopWithX),
        new WaitCommand(2.0),
        drive.run(drive::stop));
  }
}
