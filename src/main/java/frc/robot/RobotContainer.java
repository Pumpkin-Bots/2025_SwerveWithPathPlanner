// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.reduxrobotics.canand.CanandEventLoop;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.FollowPathToApriltag;
import frc.robot.commands.Intake;
import frc.robot.commands.OuttakeFirst;
import frc.robot.commands.OuttakeSecond;
import frc.robot.commands.PathfindToApriltagOffset;
import frc.robot.commands.PathfindToApriltagOffset;
import frc.robot.commands.StopIntake;
import frc.robot.commands.Throwup;
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
    
    // Assume Blue Alliance unless otherwise specified.
    // Checks for alliance color in robotperiodic and sets to -1 if red.
    // Use this to invert joystick inputs when on Red Alliance.
    // This means that when on Red or Blue team, origin is always Blue[0,0]
    // But driver drives "forward" on red or blue side to move robot forward, etc.
    public int joystickInvert = 1; 

    //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    /* private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    */
    
    //* Used for looking up AprilTag poses for pathfinding */
    FieldConstants constants = new FieldConstants();
    int m_aprilTagTarget = 6; // default to 6 for now (for testing)

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(Constants.Controllers.kJoystickId);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final CoralLauncher m_launcher = new CoralLauncher();
    private final Climber m_climber = new Climber();
    private final CANdleSystem m_CANdle = new CANdleSystem(joystick);
    private final VisionDriveSystem m_VisionDriveSystem = new VisionDriveSystem();

    /* Path follower */ 
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("Intake", new Intake(m_launcher));
        NamedCommands.registerCommand("OuttakeFirst", new OuttakeFirst(m_launcher));
        NamedCommands.registerCommand("OuttakeSecond", new OuttakeSecond(m_launcher));
        NamedCommands.registerCommand("StopIntake", new StopIntake(m_launcher));
        NamedCommands.registerCommand("FollowPathToApriltag", new FollowPathToApriltag(m_VisionDriveSystem, m_aprilTagTarget));
        NamedCommands.registerCommand("PathfindToApriltagOffset", new PathfindToApriltagOffset(m_VisionDriveSystem, m_aprilTagTarget));

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
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
                drive.withVelocityX(-velocityCurveTranslate(joystick.getLeftY()) * MaxSpeed * joystickInvert) // Drive forward with negative Y (forward)
                    .withVelocityY(-velocityCurveTranslate(joystick.getLeftX()) * MaxSpeed * joystickInvert) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Rotate with right stick X
            )
        );

        // When hoilding down POV right directional key, apply brake
        joystick.povRight().whileTrue(drivetrain.applyRequest(() -> brake));

        /*  When holding down B key, drive toward the nearest AprilTag
        joystick.b().whileTrue(drivetrain.applyRequest(() -> 
            driveRobotCentric.withVelocityY(-LimelightHelpers.getBotPose_TargetSpace(Constants.Vision.kLimelightBack)[0]*5)
            .withVelocityX(LimelightHelpers.getBotPose_TargetSpace(Constants.Vision.kLimelightBack)[2]*1)
            .withRotationalRate((-LimelightHelpers.getBotPose_TargetSpace(Constants.Vision.kLimelightBack)[4]*0.1))));  
        */

        // When pressing the B key, drive to reef apriltag 6 - need to generalize this later
        joystick.b().whileTrue(new FollowPathToApriltag(m_VisionDriveSystem, m_aprilTagTarget));

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
        
        joystick.y().onTrue(new Intake(m_launcher));
        //joystick.x().onTrue(new Outtake(m_launcher));
        joystick.a().onTrue(new OuttakeSecond(m_launcher));
        joystick.x().onTrue(new OuttakeFirst(m_launcher));
        joystick.povLeft().whileTrue(new Throwup(m_launcher));
        joystick.rightBumper().onTrue(new StopIntake(m_launcher));
        joystick.povDown().whileTrue(new ClimbDown(m_climber));
        joystick.povUp().whileTrue(new ClimbUp(m_climber));
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
