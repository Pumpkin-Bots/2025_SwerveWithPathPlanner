// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Array;
import java.util.Arrays;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final Field2d m_field = new Field2d();

  public Robot() {
    m_robotContainer = new RobotContainer();
    SmartDashboard.putData("Field", m_field);
    
    //LimelightHelpers.setLEDMode_ForceOff(Constants.Vision.kLimelightBack);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

   // System.out.println("Coral detected: %s/f".formatted(Boolean.toString(CoralSensor.isCoralDetected())));

  

  /* Update robot pose every cycle using Limelight cameras and AprilTags   */
    if (Constants.Vision.kUseLimelight) {
      // First, get your robot's current orientation from gyro
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
      
      // Set standard deviations for vision measurements
      m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));

      var m_limelightNames = Constants.Vision.kLimelightNames;
      /*
      if (LimelightHelpers.getTV(m_limelightNames[0])){
        LimelightHelpers.SetRobotOrientation(m_limelightNames[0], headingDeg, 0, 0, 0, 0, 0);
        var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_limelightNames[0]);
        if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
          // This uses X,Y values from vision and resets pose of robot. 
          m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
          System.out.println("LL " + m_limelightNames[0] + " Update: " + m_robotContainer.drivetrain.getState().Pose);
        }
      }
      else if (LimelightHelpers.getTV(m_limelightNames[1])){
        LimelightHelpers.SetRobotOrientation(m_limelightNames[1], headingDeg, 0, 0, 0, 0, 0);
        var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_limelightNames[0]);
        if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
          // This uses X,Y values from vision and resets pose of robot. 
          m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
          System.out.println("LL " + m_limelightNames[1] + " Update: " + m_robotContainer.drivetrain.getState().Pose);
        }
      }
      */
      for (String m_limelightName : m_limelightNames) {
        // Set robot orientation using gyro value 
        LimelightHelpers.SetRobotOrientation(m_limelightName, headingDeg, 0, 0, 0, 0, 0);
        
        // Get pose estimate from Limelight assumes Blue field orientation 
        var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_limelightName);
        
        if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
          // This uses X,Y values from vision and resets pose of robot. 
          m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
          //System.out.println("LL " + m_limelightName + " Update: " + m_robotContainer.drivetrain.getState().Pose);
        }
      }

      // This was the code that was added during the merge with Alex's code
      Pose2d currentPose = getCurrentPose();
      m_field.setRobotPose(currentPose);
      
      
    }
  }

  /**
   * Retrieves the current pose of the robot from the drivetrain.
   * 
   * @return The current Pose2d of the robot.
   */
  public Pose2d getCurrentPose() {
    return m_robotContainer.drivetrain.getState().Pose;
  }

  @Override
  public void disabledInit() {
    //LimelightHelpers.setLEDMode_ForceOff(Constants.Vision.kLimelightBack);
  }
  
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
    //LimelightHelpers.setLEDMode_ForceOn(Constants.Vision.kLimelightBack);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    //LimelightHelpers.setLEDMode_ForceOff(Constants.Vision.kLimelightBack);
  }
  
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //LimelightHelpers.setLEDMode_ForceOn(Constants.Vision.kLimelightBack);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    //LimelightHelpers.setLEDMode_ForceOff(Constants.Vision.kLimelightBack);
  }

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
