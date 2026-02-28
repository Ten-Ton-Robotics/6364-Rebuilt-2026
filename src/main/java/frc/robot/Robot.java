// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonHandler;

public class Robot extends TimedRobot {
  private CommandSwerveDrivetrain drivetrain = RobotContainer.m_drivetrain;
  private Command m_autonomousCommand;
  public static PhotonHandler m_vision;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    Transform3d cameraToRobot = new Transform3d(new Translation3d(-0.2286, 0.2794, 0.61876), new Rotation3d(0,0,Math.PI));
    m_vision = new PhotonHandler(drivetrain::addVisionMeasurement, "Practice_Cam" , cameraToRobot.inverse());
    // CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotInit(){
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
      }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    m_vision.periodic(); 
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
          CommandScheduler.getInstance().schedule(m_autonomousCommand); 
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
  public void simulationPeriodic() {
  }
}
