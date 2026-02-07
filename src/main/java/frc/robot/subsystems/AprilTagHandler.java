package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class AprilTagHandler {
    // Constants
    public static final List<Integer> kBlueIds = List.of(17, 28, 18, 27, 19, 20, 26, 25, 21, 24, 22, 23, 29, 30, 31, 32);
    public static final List<Integer> kRedIDs = List.of(7, 6, 8, 5, 9, 10, 4, 3, 11, 2, 12, 1, 16, 15, 14, 13);

    public AprilTagHandler() {
        SmartDashboard.putNumber("AprilTag Yaw", getYawToTargetInRadian());
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

                    SmartDashboard.putNumber("AprilTag Yaw", getYawToTargetInRadian());

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

    // This Command is purely for testing whether the AprilTag filtering works
    public Command testWhetherTheBestTargetAprilTagIsFromAlliance() {
        return new InstantCommand(() -> {
            List<PhotonPipelineResult> latestResults = Robot.m_vision.latestResults;

            try {
                PhotonPipelineResult latestResult = latestResults.get(0);

                if (latestResult.hasTargets()) {
                    PhotonTrackedTarget bestTarget = latestResult.getBestTarget();

                    System.out.println(doesTagMatchAlliance(bestTarget.getFiducialId()) ? "The April Tag matches our current alliance" : "The April Tag does NOT match our current alliance");
                } else {
                    System.out.println("The result has no target");
                }
            } catch (Exception e) {
                System.out.println("Failed to get latest result with exception: " + e.getLocalizedMessage());
            }
        });
    }

    public boolean doesTagMatchAlliance(int id) {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Blue) {
                return kBlueIds.contains(id);
            } else {
                return kRedIDs.contains(id);
            }
        } else {
            return false;
        }
    }
}
