// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)  * 0.3; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve m_drive platform */
    private final SwerveRequest.FieldCentricFacingAngle m_drive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for m_drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_controller = new CommandXboxController(0);
    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                m_drive.withVelocityX(-m_controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withTargetRateFeedforward(MaxAngularRate * m_controller.getRightX()) 
                    // .withTargetDirection(null)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the m_drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        m_controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
        m_controller.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_controller.getLeftY(), -m_controller.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_controller.back().and(m_controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_controller.back().and(m_controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_controller.start().and(m_controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_controller.start().and(m_controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        m_controller.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        m_controller.y().whileTrue(
            drivetrain.applyRequest(() -> m_drive.withTargetDirection(new Rotation2d(Math.PI/2))
            .withHeadingPID(0.1, 0.1, 0.1))
            );
    }  

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
