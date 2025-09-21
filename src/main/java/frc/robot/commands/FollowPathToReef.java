package frc.robot.commands;

import java.nio.file.Path;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionDriveSystem;



public class FollowPathToReef extends Command {
    
    int aprilTagTarget;

    private VisionDriveSystem m_VisionDriveSystem;
    private Command pathCommand;  // Store the path command
    
    public FollowPathToReef(VisionDriveSystem driveSystem, int aprilTagTarget) {
        System.out.println("CHECKPOINT 1");
        this.aprilTagTarget = aprilTagTarget;
        System.out.println("CHECKPOINT A");
        m_VisionDriveSystem = driveSystem;
        System.out.println("CHECKPOINT B");
        addRequirements(m_VisionDriveSystem);
        System.out.println("CHECKPOINT C");
        
    }
    
    @Override
    public void initialize() {
        PathPlannerPath path = null;
        System.out.println("CHECKPOINT 2");
        try{
        path = m_VisionDriveSystem.getPathToVisionTarget(aprilTagTarget);
        }
        catch(Exception e){
            System.out.println("ERROR DURING PATH CREATION" + e.getMessage());
        }

        System.out.println("CHECKPOINT 12");
        if (!(path == null)){

        
        pathCommand = AutoBuilder.followPath(path);
        System.out.println("CHECKPOINT 13");
        
        pathCommand.schedule();  // Start the path
        System.out.println("CHECKPOINT 14");
        }
        else {
            System.out.println("PATH IS NULL");
        }
    }
    @Override
    public boolean isFinished() {
        return pathCommand != null && pathCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        stop();
    }

    public void stop() {
        if (pathCommand != null) {
            pathCommand.cancel();  // Stops the path-following command
            pathCommand = null;    // Reset the command reference
        }
        m_VisionDriveSystem.stop();  // Stop the drivetrain motors
    }
}
