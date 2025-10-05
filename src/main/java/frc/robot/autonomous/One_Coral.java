package frc.robot.autonomous;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;

public class One_Coral extends Command{

    PathPlannerAuto auto;

    public One_Coral(){
        auto = new PathPlannerAuto("Coral1Auto");
    }


    
}
