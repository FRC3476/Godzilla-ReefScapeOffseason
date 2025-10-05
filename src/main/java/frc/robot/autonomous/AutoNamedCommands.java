package frc.robot.autonomous;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.end_effector.Claw;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;

public class AutoNamedCommands {

  private Superstructure superstructure;
  private Intake intake;
  private Drive drive;
  private Claw claw;
  //   private Vision vision;

  public AutoNamedCommands(Superstructure superstructure, Intake intake, Drive drive, Claw claw
      //   ,
      //   Vision vision
      ) {
    this.superstructure = superstructure;
    this.intake = intake;
    this.drive = drive;
    this.claw = claw;
    // this.vision = vision;
  }

  public void registerNamedCommands() {

    NamedCommands.registerCommand(
        "L4_aim", superstructure.setStateCommand(SuperstructureState.L4_AIM, "Aim L4"));

    NamedCommands.registerCommand(
        "L4_score",
        Commands.sequence(
            superstructure.setStateCommand(SuperstructureState.L4_SCORE, "Score L4"),
            claw.rollerFWD()));

    NamedCommands.registerCommand(
        "L4_fadeaway",
        Commands.sequence(
            claw.rollerSTOP(),
            superstructure.setStateCommand(SuperstructureState.L4_FADEAWAY, "Fadeaway L4")));

    NamedCommands.registerCommand(
        "STOW", superstructure.setStateCommand(SuperstructureState.STOW, "Stow"));

    NamedCommands.registerCommand(
        "intake_CORAL",
        Commands.parallel(
            superstructure.setStateCommand(SuperstructureState.INTAKE_CORAL, "Intake Ground Coral"),
            intake.setIntakeStateCommand(IntakeState.INTAKE)));
    // NamedCommands.registerCommand(
    //     "drive_to_coral", DriveCommands.driveToCoral(drive, vision, intake));

    // NamedCommands.registerCommand("drive_to_2ndCoralScore",
    //         // DriveCommands.pathfindtoPose(wtv pose for 2ndcoral)
    // );
  }
}
