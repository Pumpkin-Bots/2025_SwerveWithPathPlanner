package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralLauncher;
import frc.robot.subsystems.CoralSensor;
public class Outtake extends Command{
    private CoralLauncher m_launcher;
    public Outtake(CoralLauncher launcher) {
        m_launcher = launcher;
        addRequirements(m_launcher);
    }

    @Override
    public void initialize() {
        m_launcher.setIntakeWheel(1);
    }
    @Override
    public boolean isFinished() {
        return !CoralSensor.isCoralDetected();
    }

    @Override
    public void end(boolean interrupted) {
        m_launcher.setIntakeWheel(0);
    }

    public void stopOuttake() {
        m_launcher.setIntakeWheel(0);
    }
}
