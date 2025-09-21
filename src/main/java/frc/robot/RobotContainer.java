// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.reduxrobotics.canand.CanandEventLoop;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.FollowPathToReef;
import frc.robot.commands.PathfindToApriltagOffset;
import frc.robot.commands.SetLEDMode;
import frc.robot.commands.CoralIntakeStart;
import frc.robot.commands.CoralScoreSlow;
import frc.robot.commands.CoralScoreFast;
import frc.robot.commands.CoralIntakeStop;
import frc.robot.commands.CoralThrowup;
import frc.robot.commands.DriveToNearestApriltag;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralLauncher;
import frc.robot.subsystems.VisionDriveSystem;
import frc.robot.subsystems.CANdleSystem;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric();

    //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    /* private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    */
    
    //* Used for looking up AprilTag poses for pathfinding */
    int m_aprilTagTarget = 20; // default to 6 for now (for testing)

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(Constants.Controllers.kJoystickId);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final CoralLauncher m_launcher = new CoralLauncher();
    private final Climber m_climber = new Climber();
    private final CANdleSystem m_CANdle = new CANdleSystem(joystick);
    private final VisionDriveSystem m_VisionDriveSystem = new VisionDriveSystem();
    final SlewRateLimiter xLimiter = new SlewRateLimiter(2.0);
    final SlewRateLimiter yLimiter = new SlewRateLimiter(2.0);
    final SlewRateLimiter rotLimiter = new SlewRateLimiter(2.0);

    /* Path follower */ 
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("ClimbDown", new ClimbDown(m_climber));
        NamedCommands.registerCommand("ClimbUp", new ClimbUp(m_climber));
        NamedCommands.registerCommand("CoralIntakeStart", new CoralIntakeStart(m_launcher));
        NamedCommands.registerCommand("CoralIntakeStop", new CoralIntakeStop(m_launcher));
        NamedCommands.registerCommand("CoralScoreFast", new CoralScoreFast(m_launcher));
        NamedCommands.registerCommand("CoralScoreSlow", new CoralScoreSlow(m_launcher));
        NamedCommands.registerCommand("CoralThrowup", new CoralThrowup(m_launcher));
        NamedCommands.registerCommand("FollowPathToReef", new FollowPathToReef(m_VisionDriveSystem, m_aprilTagTarget));
        NamedCommands.registerCommand("PathfindToApriltagOffset", new PathfindToApriltagOffset(m_VisionDriveSystem, m_aprilTagTarget));
        NamedCommands.registerCommand("DriveToNearestApriltag", new DriveToNearestApriltag(drivetrain));

        autoChooser = AutoBuilder.buildAutoChooser("None");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
        configureCanandColor();
        
    }

    /* Modulate speed of swerve drive based on joystick input */
    private double velocityCurveTranslate(double joystickInput) { 
        if(joystickInput > 0){
          return Math.pow(joystickInput, 2.9);
        } else if (joystickInput < 0){
          return -Math.pow(-joystickInput, 2.9);
        } else {
          return 0;
        }
      }
    
    
    

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-velocityCurveTranslate(joystick.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-velocityCurveTranslate(joystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Rotate with right stick X
            )
        );

        // When hoilding down POV right directional key, apply brake
        joystick.povRight().whileTrue(drivetrain.applyRequest(() -> brake));

        //  When holding down B key, drive toward the nearest AprilTag
          
        joystick.b().whileTrue(
            
            Commands.either(
                drivetrain.applyRequest(() -> 
                    driveRobotCentric.withVelocityY(-LimelightHelpers.getBotPose_TargetSpace(Constants.Vision.kLimelightBack)[0] * 5)
                    .withVelocityX(LimelightHelpers.getBotPose_TargetSpace(Constants.Vision.kLimelightBack)[2] * 1)
                    .withRotationalRate(-LimelightHelpers.getBotPose_TargetSpace(Constants.Vision.kLimelightBack)[4] * 0.1)
                ),
                Commands.none()
                ,
                () -> LimelightHelpers.getTV(Constants.Vision.kLimelightBack)
            )
           
        );

        
        /* 
        joystick.b().whileTrue(
            new RunCommand(() -> {
                double tx = LimelightHelpers.getTX(Constants.Vision.kLimelightBack);
                double[] pose = LimelightHelpers.getBotPose_TargetSpace(Constants.Vision.kLimelightBack);

                double xSpeed = xLimiter.calculate(pose[2]); // forward/backward (Z)
                double ySpeed = yLimiter.calculate(-pose[0]); // left/right (X)
                double rotSpeed = rotLimiter.calculate(-0.1 * tx); // rotation

                drivetrain.applyRequest(() ->
                    driveRobotCentric
                        .withVelocityX(xSpeed)
                        .withVelocityY(ySpeed)
                        .withRotationalRate(rotSpeed)
                );
            }, drivetrain)
        );
        */
        /*
        joystick.b().whileTrue(
            drivetrain.applyRequest(() -> 
                    driveRobotCentric.withVelocityY(-LimelightHelpers.getBotPose_TargetSpace(Constants.Vision.kLimelightBack)[0] * 4)
                    .withVelocityX(LimelightHelpers.getBotPose_TargetSpace(Constants.Vision.kLimelightBack)[2] * 1.25)
                    .withRotationalRate(-LimelightHelpers.getBotPose_TargetSpace(Constants.Vision.kLimelightBack)[4] * 0.05)   //withRotationalRate(-LimelightHelpers.getBotPose_TargetSpace(Constants.Vision.kLimelightBack)[4] * 0.05)
                
        ));
        */
        
        

        //joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
        //    forwardStraight.withVelocityX(0.5).withVelocityY(0))
        //);
        //joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
        //    forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        //);

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        joystick.povDown().whileTrue(new ClimbDown(m_climber));
        joystick.povUp().whileTrue(new ClimbUp(m_climber));

        joystick.y().onTrue(new CoralIntakeStart(m_launcher));
        joystick.rightBumper().onTrue(new CoralIntakeStop(m_launcher));
        //joystick.x().onTrue(new Outtake(m_launcher));
        
        joystick.a().onTrue(new CoralScoreFast(m_launcher));
        joystick.x().onTrue(new CoralScoreSlow(m_launcher));
        
        joystick.povLeft().whileTrue(new CoralThrowup(m_launcher).withTimeout(2));

        //joystick.b().onTrue(new SetLEDMode(Constants.Vision.kLimelightBack));
        // Drive to reef apriltag 6 - need to generalize this later
        //joystick.b().onTrue(new FollowPathToReef(m_VisionDriveSystem, m_aprilTagTarget));
        

        drivetrain.registerTelemetry(logger::telemeterize);
        
    }

    public void configureCanandColor() {
        // This will start Redux CANLink manually for Java
        CanandEventLoop.getInstance();
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
        
    }
}
