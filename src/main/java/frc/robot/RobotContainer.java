// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Feed;

public class RobotContainer {
    private boolean isSnapToggleOn = false;

    // Subsystems
    public static final Shooter m_Shooter = new Shooter();
    public static final Feed m_Feed = new Feed();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)  * 0.3; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve m_drive platform */
    private final SwerveRequest.FieldCentricFacingAngle m_drive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for m_drive motors
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

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
            m_drivetrain.applyRequest(() ->
                getCurrentSwerveRequest()
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the m_drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            m_drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        //April Tag Snapping
        m_controller.a().onTrue(toggleAprilTagSnapCommand());
        
        //Shooter and Feed Control 
        m_controller.x().onTrue(m_Shooter.toggleShooting());
        m_controller.povDown().onTrue(m_Shooter.changeSpeed(5));
        m_controller.povUp().onTrue(m_Shooter.changeSpeed(-5));
        m_controller.leftTrigger().onTrue(m_Feed.intake()); 
        m_controller.leftTrigger().onFalse(m_Feed.stop()); 

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

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    private Command toggleAprilTagSnapCommand() {
        return new InstantCommand(() -> { isSnapToggleOn = !isSnapToggleOn; });
    }

    public double getYawToTargetInRadian() {
        List<PhotonPipelineResult> latestResults = Robot.m_vision.latestResults;
        
        try {
            PhotonPipelineResult latestResult = latestResults.get(0);

            if (latestResult.hasTargets()) {
                PhotonTrackedTarget bestTarget = latestResult.getBestTarget();

                if (doesTagMatchAlliance(bestTarget.getFiducialId())) {
                    double yaw = bestTarget.getYaw();
                    double yawInRadian = Units.degreesToRadians(yaw);

                    return yawInRadian;
                } else {
                    return 0;
                }
            } else {
                return 0;
            }   
        } catch (Exception e) {
            return 0;
        }
    }

    public SwerveRequest.FieldCentricFacingAngle getCurrentSwerveRequest() {
        if (isSnapToggleOn) {
            return m_drive
                .withVelocityX(-m_controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-m_controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withTargetDirection(new Rotation2d(getYawToTargetInRadian()))
                .withHeadingPID(2, 1, 1);
        } else {
            return m_drive
                .withVelocityX(-m_controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-m_controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withTargetRateFeedforward(MaxAngularRate * m_controller.getRightX());
        }
    }

    public boolean doesTagMatchAlliance(int id) {
        final List<Integer> blueIds = List.of(17, 28, 18, 27, 19, 20, 26, 25, 21, 24, 22, 23, 29, 30, 31, 32);
        final List<Integer> redIDs = List.of(7, 6, 8, 5, 9, 10, 4, 3, 11, 2, 12, 1, 16, 15, 14, 13);

        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Blue) {
                return blueIds.contains(id);
            } else {
                return redIDs.contains(id);
            }
        } else {
            return false;
        }
     }
}
