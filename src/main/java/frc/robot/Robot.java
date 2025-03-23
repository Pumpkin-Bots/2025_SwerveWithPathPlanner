// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CoralSensor;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

   // System.out.println("Coral detected: %s/f".formatted(Boolean.toString(CoralSensor.isCoralDetected())));

  /* Update robot pose every cycle using Limelight cameras and AprilTags   */
    if (Constants.Vision.kUseLimelight) {

      // First, tell Limelight your robot's current orientation
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
      LimelightHelpers.SetRobotOrientation(Constants.Vision.kLimelightBack, headingDeg, 0, 0, 0, 0, 0);
      LimelightHelpers.SetRobotOrientation(Constants.Vision.kLimelightFront, headingDeg, 0, 0, 0, 0, 0);

      /* Get pose estimate from Limelight assumes Blue field orientation */
      var llMeasurementBack = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Vision.kLimelightBack);
      var llMeasurementFront = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Vision.kLimelightFront);

      /* This uses X,Y values from vision and resets pose of robot. */
      /* Uses Back camera AprilTags if available, otherwise uses Front camera AprilTags */
      if (llMeasurementBack != null && llMeasurementBack.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurementBack.pose, llMeasurementBack.timestampSeconds);
        System.out.println("LLBack Pose Update: " + m_robotContainer.drivetrain.getState().Pose);
      } else if (llMeasurementFront != null && llMeasurementFront.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurementFront.pose, llMeasurementFront.timestampSeconds);
        System.out.println("LLFront Pose Update: " + m_robotContainer.drivetrain.getState().Pose);
      }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
