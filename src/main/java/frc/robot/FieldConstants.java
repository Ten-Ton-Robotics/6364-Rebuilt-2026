package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants {
    public static final Translation2d kBlueHub = new Translation2d(4.6, 4.005);
    public static final Translation2d kRedHub = new Translation2d(11.915394, 4.034536);

    /** 
     * @return Returns a Translation2d of the matching hub
     */
    public static Translation2d getHubPositionMatchingAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            return alliance.get() == Alliance.Blue ? kBlueHub : kRedHub;
        } else {
            System.out.println("FieldConstants tried to access Alliance but failed; returning Blue by default.");
            return kBlueHub;
        }
    }
}
