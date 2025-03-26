package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CoralLauncher;

public class StopIntake extends InstantCommand {
    public StopIntake(CoralLauncher launcher) {
        super(() -> {
            launcher.setIntakeWheel(0);
            launcher.setOuttakeWheel(0);
        }, launcher);
    }
}
