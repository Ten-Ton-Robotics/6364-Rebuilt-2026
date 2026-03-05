package frc.robot;

import java.util.Optional;
import java.util.TreeMap;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldUtil {
    public static final Translation2d kBlueHub = new Translation2d(4.6, 4.03);
    public static final Translation2d kRedHub = new Translation2d(11.915394, 4.034536);
    private static TreeMap<Double, Double> rangeMap = new TreeMap<>();   
    
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
    
    private static void fillTable(){
        rangeMap.put(1.823, 43.0);
        rangeMap.put(2.182, 45.0);
        rangeMap.put(2.670, 50.0);
        rangeMap.put(3.157, 51.0);
        rangeMap.put(3.573, 53.0); 
        rangeMap.put(4.021, 58.0); 
        rangeMap.put(4.436, 60.0);
        rangeMap.put(5.185, 64.0);  
    }

    public static double getPowerFromRange(){
        if (rangeMap.isEmpty()){
            fillTable();
        }
        double distance = FieldUtil.GetHubDistance();
        SmartDashboard.putNumber("Hub Distance", distance); 
        double rangeDelta = rangeMap.get(rangeMap.firstKey()); 
        double rangeKey = rangeMap.firstKey();

        for(double range: rangeMap.keySet()){
            double tempDelta = distance - range;
            if(Math.abs(tempDelta) < Math.abs(rangeDelta)){
                rangeDelta = tempDelta; 
                rangeKey = range; 
            }
        }
        
        //If the delta is positive then the distance is greater than the point 
        //Set to var as higherkey can return a double or null 
           
        double maxRange = (rangeDelta > 0) ? Optional.ofNullable(rangeMap.higherKey(rangeKey)).orElse(rangeKey) : rangeKey; 
        double minRange = (rangeDelta > 0) ? rangeKey : Optional.ofNullable(rangeMap.lowerKey(rangeKey)).orElse(rangeKey); 

        if(maxRange == minRange){
            return rangeMap.get(rangeKey); 
        }
         
        //How much more the distance is than the lower range divided by distance between the two ranges  
        double percentOfRange = (distance - minRange)/(maxRange - minRange);             

        //What the difference between the larger and smaller power
        double powerDifference = rangeMap.get(maxRange) - rangeMap.get(minRange);  
        
        //The smaller power plus an extra based on far it is from the next range point 
        double power = rangeMap.get(minRange) + (powerDifference * percentOfRange);  
        SmartDashboard.putNumber("Reccomended Power", power); 
        return power;   
    }

    public static Rotation2d getAngleToHub() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        Translation2d hubPosition = FieldUtil.getHubPositionMatchingAlliance();

        Translation2d robotPosition = RobotContainer.m_drivetrain.getPose().getTranslation();

        double xDifference = hubPosition.getX() - robotPosition.getX();
        double yDifference = hubPosition.getY() - robotPosition.getY();

        Rotation2d hubAngle = alliance.get() == Alliance.Blue ? new Rotation2d(Math.atan2(yDifference, xDifference) + Math.PI) : new Rotation2d(Math.atan2(yDifference, xDifference)); 
        SmartDashboard.putNumber("Hub Angle", hubAngle.getRadians()); 
        return hubAngle; 
    }

    public static double GetHubDistance(){
        Translation2d hubPosition = FieldUtil.getHubPositionMatchingAlliance();
        Translation2d robotPosition = RobotContainer.m_drivetrain.getPose().getTranslation();

        double xDifference = hubPosition.getX() - robotPosition.getX();
        double yDifference = hubPosition.getY() - robotPosition.getY();
        
        double hubDistance = Math.sqrt(Math.pow(xDifference, 2) + Math.pow(yDifference, 2)); 

        return hubDistance;
    }
}
