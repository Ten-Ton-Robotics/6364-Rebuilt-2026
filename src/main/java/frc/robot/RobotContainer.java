// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class RobotContainer {
    public boolean isHubSnappingOn = false;
    public Rotation2d hubTargetAngle = new Rotation2d(0.0);

    // Subsystems
    public static final Shooter m_Middle_Shooter = new Shooter(44, "Middle");
    public static final Shooter m_Left_Shooter = new Shooter(37, "Left");
    public static final Shooter m_Right_Shooter = new Shooter(35, "Right");

    public static final Feed m_Feed = new Feed(13);
    public static final Indexer m_Indexer = new Indexer(21); 
    public static final Intake m_Intake = new Intake(); 

    public static final AprilTagHandler m_AprilTagHandler = new AprilTagHandler();
    
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)  * 0.3; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(.50).in(RadiansPerSecond); // 1/2 of a rotation per second max angular velocity

    public double hubDistance = 0; 
    /* Setting up bindings for necessary control of the swerve m_drive platform */
    private final SwerveRequest.FieldCentricFacingAngle m_drive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.001) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for m_drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_controller = new CommandXboxController(0);
    public final static CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
    
    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            m_drivetrain.applyRequest(() -> {
                var baseDrive = m_drive
                    .withVelocityX(-m_controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_controller.getLeftX() * MaxSpeed); // Drive left with negative X (left)

                SmartDashboard.putBoolean("Hub Snap Toggle", isHubSnappingOn);

                if (isHubSnappingOn) {
                    return baseDrive
                        .withTargetDirection(hubTargetAngle)
                        .withHeadingPID(3, 0, 0)
                        .withMaxAbsRotationalRate(MaxAngularRate);
                } else {
                    double rightJoyStick = Math.abs(m_controller.getRightX()) < 0.1 ? 0 : m_controller.getRightX() ;
                    return baseDrive
                        .withTargetRateFeedforward(MaxAngularRate * rightJoyStick)
                        .withHeadingPID(0, 0, 0);
                }
            })
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the m_drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            m_drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Hub Snapping
        m_controller.a().onTrue(toggleSnappingToHub());

        // Sequential Commands
        m_controller.y().onTrue(shoot());
        
        // Shooter toggle
        m_controller.x().onTrue(m_Middle_Shooter.toggleShooting());
        m_controller.x().onTrue(m_Right_Shooter.toggleShooting());
        m_controller.x().onTrue(m_Left_Shooter.toggleShooting());

        // Shooter speed control
        m_controller.povUp().onTrue(m_Middle_Shooter.changeSpeed(true));
        m_controller.povDown().onTrue(m_Middle_Shooter.changeSpeed(false));
        m_controller.povUp().onTrue(m_Left_Shooter.changeSpeed(true));
        m_controller.povDown().onTrue(m_Left_Shooter.changeSpeed(false));
        m_controller.povUp().onTrue(m_Right_Shooter.changeSpeed(true));
        m_controller.povDown().onTrue(m_Right_Shooter.changeSpeed(false));
        
        // Shooter speed precise control
        m_controller.leftTrigger().onTrue(m_Middle_Shooter.perciseControl(1)); 
        m_controller.leftTrigger().onFalse(m_Middle_Shooter.perciseControl(5)); 
        m_controller.leftTrigger().onTrue(m_Right_Shooter.perciseControl(1)); 
        m_controller.leftTrigger().onFalse(m_Right_Shooter.perciseControl(5)); 
        m_controller.leftTrigger().onTrue(m_Left_Shooter.perciseControl(1)); 
        m_controller.leftTrigger().onFalse(m_Left_Shooter.perciseControl(5)); 

        // Feed
        m_controller.rightTrigger().onTrue(m_Feed.intake()); 
        m_controller.rightTrigger().onFalse(m_Feed.stop()); 
        
        // Indexer
        m_controller.rightBumper().onTrue(m_Indexer.intake());
        m_controller.rightBumper().onFalse(m_Indexer.stop());
        
        // Intake
        // m_controller.rightBumper().onTrue(m_Intake.intake()); 
        // m_controller.rightBumper().onFalse(m_Intake.stop()); 
        

        // m_controller.b().onTrue(m_drivetrain.FindAndFollowPath()); 
        
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_controller.back().and(m_controller.y()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
        m_controller.back().and(m_controller.x()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
        m_controller.start().and(m_controller.y()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
        m_controller.start().and(m_controller.x()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        m_controller.leftBumper().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric()));

        m_drivetrain.registerTelemetry(logger::telemeterize);
    }  

    private Command shoot() {
        return new SequentialCommandGroup(
            m_Left_Shooter.startShooting(),
            m_Middle_Shooter.startShooting(),
            m_Right_Shooter.startShooting(),

            new InstantCommand(() -> {
                while (!areAllShootersWithinTargetSpeedRange()) {
                    new WaitCommand(0.2);
                } 
            }),

            m_Feed.intake(),
            new WaitCommand(0.1),
            m_Indexer.intake()
        );
    }

    private Boolean areAllShootersWithinTargetSpeedRange() {
        Double minimumSpeed = m_Middle_Shooter.targetSpeed - 1;
        Double maximumSpeed = m_Middle_Shooter.targetSpeed + 1;

        Boolean isLeftShooterWithinRange = (minimumSpeed <= m_Left_Shooter.getCurrentMotorRPS() && m_Left_Shooter.getCurrentMotorRPS() <= maximumSpeed);
        Boolean isMiddleShooterWithinRange = (minimumSpeed <= m_Middle_Shooter.getCurrentMotorRPS() && m_Middle_Shooter.getCurrentMotorRPS() <= maximumSpeed);
        Boolean isRighttShooterWithinRange = (minimumSpeed <= m_Right_Shooter.getCurrentMotorRPS() && m_Right_Shooter.getCurrentMotorRPS() <= maximumSpeed);

        return (isLeftShooterWithinRange && isMiddleShooterWithinRange && isRighttShooterWithinRange);
    }

    private Command toggleSnappingToHub() {
        return new InstantCommand(() -> {
            hubTargetAngle = getAngleToHub();
            isHubSnappingOn = !isHubSnappingOn;
        });
    }

    private Rotation2d getAngleToHub() {
        Translation2d hubPosition = FieldConstants.getHubPositionMatchingAlliance();

        Pose2d currentPose = m_drivetrain.getPose();
        Translation2d robotPosition = currentPose.getTranslation();

        double xDifference = hubPosition.getX() - robotPosition.getX();
        double yDifference = hubPosition.getY() - robotPosition.getY();

        hubDistance = Math.sqrt(Math.pow(xDifference, 2) + Math.pow(yDifference, 2)); 
        SmartDashboard.putNumber("Hub Distance", hubDistance); 

        Rotation2d hubAngle = new Rotation2d(Math.atan2(yDifference, xDifference) + Math.PI); 
        SmartDashboard.putNumber("Hub Angle", hubAngle.getRadians()); 
        return hubAngle;
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
