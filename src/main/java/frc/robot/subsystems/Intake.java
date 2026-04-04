package frc.robot.subsystems;

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

public class Intake extends SubsystemBase {
    // Constants
    private static final CANBus kMotorBus = new CANBus("CANCAN");

    private static final int kMotorID = 34; //Get motor ID from TunerX put that one here
    private static double TargetSpeed = 75; 
    private static double MaxSpeed = 75;

    // Motor
    private final TalonFX m_motor = new TalonFX(kMotorID, kMotorBus);

    // Motor Output
    private final VelocityVoltage m_output = new VelocityVoltage(TargetSpeed);

    // Toggle Boolean
    private boolean isOn = false;

    public Intake() {
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
                    .withStatorCurrentLimit(Amps.of(45))
                    .withStatorCurrentLimitEnable(true)
            )
        .withSlot0(slot0Configs)
        .withMotorOutput(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
            );
        m_motor.getConfigurator().apply(motorConfig);
        m_motor.setNeutralMode(NeutralModeValue.Coast);

    }

    // Commands

    public Command toggleIntaking() {
        return this.runOnce(() -> {
            isOn = !isOn;
            SmartDashboard.putBoolean("Intake Is On:", isOn); 
            if (isOn) {
                setMotorSpeed(TargetSpeed);
            } else {
                m_output.Velocity = 0; 
                m_motor.setControl(m_output);
            }
        });
    }
    public Command intake() {
        return this.runOnce(() -> {
            setMotorSpeed(TargetSpeed);
        });
    }

     public Command toggleIntakingInverse() {
        return this.runOnce(() -> {
            isOn = !isOn;
            SmartDashboard.putBoolean("Intake Is On:", isOn); 
            if (isOn) {
                setMotorSpeed(-TargetSpeed);
            } else {
                m_output.Velocity = 0; 
                m_motor.setControl(m_output);
            }
        });
    }

    public Command stop(){
        return this.runOnce(() -> {
            stopMotor();
        });
    }

    // Functions
    private void setMotorSpeed(double new_speed) {
        if(new_speed < -(TargetSpeed -1)){ 
            new_speed = -TargetSpeed;
        }

        if(new_speed > MaxSpeed){
            new_speed = MaxSpeed;
        }


        isOn = true; 
        m_output.Velocity = new_speed; 
        m_motor.setControl(m_output);
        m_motor.setNeutralMode(NeutralModeValue.Brake);
        
        if (TargetSpeed == 0.0) {
            isOn = false; 
            stopMotor();
        }
    }

    private void stopMotor() {
        m_motor.setControl(new StaticBrake());
    }

    public boolean getIsOn(){
        return isOn;
    }
    
}

