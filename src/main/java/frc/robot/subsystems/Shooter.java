package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    // Constants
    private final CANBus kMotorBus = new CANBus("CANCAN"); 
    private final int kMotorID;
    private final TalonFX m_motor;
    public final String kname; 

    //Speed Variables
    public double targetSpeed = 45;
    private double maxSpeed = 65;
    private double defaultSpeedChange = 5;


    // Motor Output
    private final VelocityVoltage m_output = new VelocityVoltage(targetSpeed);

    // Toggle Boolean
    public boolean isOn = false;

    public Shooter(int id, String name) {
        kMotorID = id; 
        kname = name; 
        m_motor = new TalonFX(kMotorID, kMotorBus);

        // Configure PID/feedforward gains for velocity control
        var slot0Configs = new Slot0Configs()
            .withKP(0.1)    // Proportional gain - adjust as needed
            .withKI(0.0)    // Integral gain
            .withKD(0.0)    // Derivative gain
            .withKS(0.0)    // Static friction feedforward
            .withKV(0.12);  // Velocity feedforward - tune this value

        var motorConfig = new TalonFXConfiguration()
        .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(80))
                    .withStatorCurrentLimitEnable(true)
            )
        .withSlot0(slot0Configs)
        .withMotorOutput(
            new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

        m_motor.getConfigurator().apply(motorConfig);
        m_motor.setNeutralMode(NeutralModeValue.Coast);
        SmartDashboard.putNumber(kname + "Shooter Target (RPS)", targetSpeed);
        
    }

    // Commands
    public Command toggleShooting() {
        return this.runOnce(() -> {
            isOn = !isOn;

            if (isOn) {
                setMotorSpeed(targetSpeed);
                SmartDashboard.putNumber(kname + "Shooter Target (RPS)", targetSpeed);
            } else {
                m_output.Velocity = 0; 
                m_motor.setControl(m_output);
            }
        });
    }

    public Command startShooting() {
        return this.runOnce(() -> {
            if (!isOn) {
                setMotorSpeed(targetSpeed);
            }
        });
    }

    public Command stopShooting() {
        return this.runOnce(() -> {
            if (isOn) {
                m_output.Velocity = 0; 
                m_motor.setControl(m_output);
            }
        });
    }

    /**
     * Sets the speed of the motor.
     * @param new_speed The new speed of the motor. Gets capped between zero and the max speed.    
     */
    private void setMotorSpeed(double new_speed) {
        targetSpeed = speedCap(new_speed);
                
        m_output.Velocity = targetSpeed; 
        m_motor.setControl(m_output);
        m_motor.setNeutralMode(NeutralModeValue.Coast);
    }

    private double speedCap(double new_speed){
        if(new_speed < 0.0){ 
            new_speed = 0.0;
        }

        if(new_speed > maxSpeed){
            new_speed = maxSpeed;
        }
        return new_speed; 
    }

    public double getCurrentMotorRPS(){
        return m_motor.getVelocity().getValueAsDouble();
    }

    /**
     * Changes the speed of the motor. 
     * @param difference How much you want to speed the motor up. Positive number speeds up motor and negative number slows down motor  
     */
    public Command changeSpeed(double difference) {
        return this.runOnce(() -> {
            double new_speed = targetSpeed - difference; 
            setMotorSpeed(new_speed);   
        });   
    }

    /**
     * Changes the speed of the motor. 
     * @param difference How much you want to speed the motor up. Positive number speeds up motor and negative number slows down motor  
     */
    public Command changeSpeed(Boolean SpeedUp) {
        return this.runOnce(() -> {
            int SpeedChanger = SpeedUp ? 1 : -1 ;
            double new_speed = targetSpeed + (defaultSpeedChange * SpeedChanger);

            new_speed = speedCap(new_speed);
            SmartDashboard.putNumber(kname + "Shooter Target (RPS)", new_speed);
             
            if (isOn) { 
                setMotorSpeed(new_speed); 
            } else{
                targetSpeed = new_speed;
            }
        });   
    }

    /**
     * Changes the increment of the motor changes 
     * @param change how much the speed should change by  
     */
    public Command changeShooterSpeedDifference(int change) {
        return this.runOnce(() -> { 
               defaultSpeedChange = change; 
        });   
    }

    /**
     * Sets the speed of the motor.
     * @param speed Speed of the motor in RPS 
     */
    public Command toggleWithSetShooterSpeed(double speed){
        return this.runOnce(() -> { 
            isOn = !isOn;
            if (isOn) {
                System.out.println( kname + "Motor toggled at " + speed + " power");
                m_output.Velocity = speedCap(speed);
                m_motor.setControl(m_output);
            } else {
                m_output.Velocity = 0; 
                m_motor.setControl(m_output);
            }
        }); 
    }

    /**
     * Sets the speed of the motor.
     * @param speed Speed of the motor in RPS 
     */
    public Command toggleWithSetShooterSpeed(DoubleSupplier speedSup){
        return this.runOnce(() -> { 
            isOn = !isOn;
            if (isOn) {
                m_output.Velocity = speedCap(speedSup.getAsDouble());
                m_motor.setControl(m_output);
            } else {
                m_output.Velocity = 0; 
                m_motor.setControl(m_output);
            }
        }); 
    }

}
