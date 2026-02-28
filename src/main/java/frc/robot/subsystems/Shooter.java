package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
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
                    .withStatorCurrentLimit(Amps.of(120))
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
            } else {
                stopMotor();
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

    /**
     * Sets the speed of the motor. Note that we want the motor to spin backwards so the speed should be negative.  
     * @param new_speed The new speed of the motor. Gets capped between zero and the max speed.    
     */
    private void setMotorSpeed(double new_speed) {
        if(new_speed < 0.0){ 
            new_speed = 0.0;
        }

        if(new_speed > maxSpeed){
            new_speed = maxSpeed;
        }

        targetSpeed = new_speed;
        
        SmartDashboard.putNumber(kname + "Shooter Target (RPS)", targetSpeed);
        
        m_output.Velocity = targetSpeed; 
        m_motor.setControl(m_output);
        m_motor.setNeutralMode(NeutralModeValue.Coast);
    }

    private void stopMotor() {
        m_motor.setControl(new StaticBrake());
    }

    public double getCurrentMotorRPS(){
        return m_motor.getVelocity().getValueAsDouble();
    }

    /**
     * Changes the speed of the motor. Note currently starts the motor on when called. 
     * @param difference How much you want to speed the motor up. Positive number speeds up motor and negative number slows down motor  
     */
    public Command changeSpeed(double difference) {
        return this.runOnce(() -> {
            double new_speed = targetSpeed - difference; 
            setMotorSpeed(new_speed);   
        });   
    }

    /**
     * Changes the speed of the motor. Note currently starts the motor on when called. 
     * @param difference How much you want to speed the motor up. Positive number speeds up motor and negative number slows down motor  
     */
    public Command changeSpeed(Boolean SpeedUp) {
        return this.runOnce(() -> {
            int SpeedChanger = SpeedUp ? 1 : -1 ;
            double new_speed = targetSpeed + (defaultSpeedChange * SpeedChanger);
            if (isOn) { setMotorSpeed(new_speed); }
        });   
    }

    /**
     * Changes the speed of the motor. Note currently starts the motor on when called. 
     * @param difference How much you want to speed the motor up. Positive number speeds up motor and negative number slows down motor  
     */
    public Command changeShooterSpeedDifference(int change) {
        return this.runOnce(() -> { 
               defaultSpeedChange = change; 
        });   
    }
}
