// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
    public boolean isSequentialShootingOn = false;

    // Subsystems
    public static final Shooter m_Middle_Shooter = new Shooter(44, "Middle");
    public static final Shooter m_Left_Shooter = new Shooter(37, "Left");
    public static final Shooter m_Right_Shooter = new Shooter(35, "Right");

    public static final Feed m_Feed = new Feed(13);
    public static final Indexer m_Indexer = new Indexer(21);
    public static final Intake m_Intake = new Intake();
    public static final Pivot m_pivot = new Pivot();

    public static final AprilTagHandler m_AprilTagHandler = new AprilTagHandler();
    
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 1/2 of a rotation per second max angular velocity

    private double recommendedPower = 0;


    /* Setting up bindings for necessary control of the swerve m_drive platform */
    private final SwerveRequest.FieldCentricFacingAngle m_drive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.01) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for m_drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_controller = new CommandXboxController(1);
    private final CommandXboxController m_test_controller = new CommandXboxController(0); 
    public final static CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
    private final SendableChooser<Command> autoChooser; 


    public RobotContainer() {
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser(); 
        SmartDashboard.putData("Auto Chooser", autoChooser); 

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
                        .withHeadingPID(5, 0, 0) //5 is figured experimentally 
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
        m_test_controller.y().onTrue(m_Indexer.putOutIntake());
        m_test_controller.y().onFalse(m_Indexer.stop()); 

        m_test_controller.a().onTrue(m_pivot.pivot()); 
        m_test_controller.a().onFalse(m_pivot.stop()); 

        
        // Shooter toggle
        m_test_controller.x().onTrue(toggleShooting());
        m_controller.x().onTrue(toggleShooting());


        //Shooter Set Speed 
        m_controller.b().onTrue(changeRecommendedPower());
        m_test_controller.b().onTrue(changeRecommendedPower()); 

        // Shooter speed control
        m_test_controller.povUp().onTrue(changeShooterSpeed(true));
        m_test_controller.povDown().onTrue(changeShooterSpeed(false));
        
        // Shooter speed precise control
        m_test_controller.leftTrigger().onTrue(changeShooterSpeedDifference(1));
        m_test_controller.leftTrigger().onFalse(changeShooterSpeedDifference(5));

        m_controller.leftTrigger().onTrue(changeShooterSpeedDifference(1));
        m_controller.leftTrigger().onFalse(changeShooterSpeedDifference(5));
        // Feed
        m_test_controller.rightTrigger().onTrue(m_Feed.intake()); 
        m_test_controller.rightTrigger().onFalse(m_Feed.stop()); 
        
            
        
        // Intake
        m_controller.rightBumper().onTrue(toggleIntaking());
        m_controller.leftBumper().onTrue(toggleIntakingInverse());
        

        // m_controller.b().onTrue(m_drivetrain.FindAndFollowPath()); 
        
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_controller.back().and(m_controller.y()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
        m_controller.back().and(m_controller.x()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
        m_controller.start().and(m_controller.y()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
        m_controller.start().and(m_controller.x()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));


        m_drivetrain.registerTelemetry(logger::telemeterize);

        NamedCommands.registerCommand("Turn on Intake", m_Intake.intake());
        NamedCommands.registerCommand("Stop Intake", m_Intake.stop());
        NamedCommands.registerCommand("Toggle Shooter with Speed", toggleShooterWithSpeed(50));
        NamedCommands.registerCommand("Start Feed", m_Feed.intake());
        NamedCommands.registerCommand("Stop Feed", m_Feed.stop());
    }  

    private Command toggleSequentialShoot() {
        return new InstantCommand(() -> {
            isSequentialShootingOn = !isSequentialShootingOn;

            if (isSequentialShootingOn) {
                sequentialShootingCommand.execute();
            } else {
                if (!sequentialShootingCommand.isFinished()) {
                    sequentialShootingCommand.end(true);
                }
                m_Left_Shooter.stopShooting();
                m_Middle_Shooter.stopShooting();
                m_Right_Shooter.stopShooting();
                m_Intake.stop();
                m_Feed.stop();
            }
        });
    }



    private Command toggleShooting() {
        return new ParallelCommandGroup(
            m_Middle_Shooter.toggleShooting(),
            m_Left_Shooter.toggleShooting(),
            m_Right_Shooter.toggleShooting()
        );
    }

    private Command toggleIntaking() {
        return new ParallelCommandGroup(
            m_Indexer.toggleIntaking(),
            m_Intake.toggleIntaking()
        );
    }

     private Command toggleIntakingInverse() {
        return new ParallelCommandGroup(
            m_Indexer.toggleIntaking(),
            m_Intake.toggleIntakingInverse()
        );
    }


    private Command changeShooterSpeed(boolean speedUp) {
        return new ParallelCommandGroup(
            m_Middle_Shooter.changeSpeed(speedUp),
            m_Left_Shooter.changeSpeed(speedUp),
            m_Right_Shooter.changeSpeed(speedUp)
        );
    }

    /**
     * Changes the value that shooter uses to change the target speed
     * @param difference The difference you want to use
     * @return A ParallelCommandGroup
     */
    private Command changeShooterSpeedDifference(int difference) {
        return new ParallelCommandGroup(
            m_Middle_Shooter.changeShooterSpeedDifference(difference),
            m_Left_Shooter.changeShooterSpeedDifference(difference),
            m_Right_Shooter.changeShooterSpeedDifference(difference)
        );
    }


    public Command stopPushingIntake(){
        return new ParallelCommandGroup(
            m_pivot.stop(), 
            m_Indexer.stop()
        ); 
    }

    private Command changeRecommendedPower(){
        return new InstantCommand(() -> {
            recommendedPower = FieldUtil.getPowerFromRange();
            Commands.print("New power:" + recommendedPower);
        });
    }

    private Command toggleSnappingToHub() {
        return new InstantCommand(() -> {
            hubTargetAngle = FieldUtil.getAngleToHub();
            isHubSnappingOn = !isHubSnappingOn;
            FieldUtil.getPowerFromRange(); 
            FieldUtil.GetHubDistance(); 
        });
    }
    
    
    private Command toggleShooterWithSpeed(double Speed){
        return new ParallelCommandGroup(
            m_Left_Shooter.toggleWithSetShooterSpeed(Speed), 
            m_Middle_Shooter.toggleWithSetShooterSpeed(Speed),
            m_Right_Shooter.toggleWithSetShooterSpeed(Speed)
        ); 
    } 

        private Command toggleShooterFromRange(){
        return new SequentialCommandGroup(
            changeRecommendedPower(), 
            toggleShooterWithSpeed(recommendedPower)
        ); 
    } 

    SequentialCommandGroup sequentialShootingCommand = new SequentialCommandGroup(
        
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


    /** Gets the middle shooter target speed and checks if all of the shooters are with 1 rps of the middle shooter target speed
     * @return Returns true if all shooters are within 1 rps of middle target speed otherwise returns false
    */
    private Boolean areAllShootersWithinTargetSpeedRange() {
        Double minimumSpeed = m_Middle_Shooter.targetSpeed - 1;
        Double maximumSpeed = m_Middle_Shooter.targetSpeed + 1;

        Boolean isLeftShooterWithinRange = (minimumSpeed <= m_Left_Shooter.getCurrentMotorRPS() && m_Left_Shooter.getCurrentMotorRPS() <= maximumSpeed);
        Boolean isMiddleShooterWithinRange = (minimumSpeed <= m_Middle_Shooter.getCurrentMotorRPS() && m_Middle_Shooter.getCurrentMotorRPS() <= maximumSpeed);
        Boolean isRighttShooterWithinRange = (minimumSpeed <= m_Right_Shooter.getCurrentMotorRPS() && m_Right_Shooter.getCurrentMotorRPS() <= maximumSpeed);

        return (isLeftShooterWithinRange && isMiddleShooterWithinRange && isRighttShooterWithinRange);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
