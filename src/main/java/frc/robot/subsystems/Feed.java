package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feed extends SubsystemBase {
    // Constants
    private final CANBus kMotorBus = new CANBus("CANCAN");
    private final int kMotorID;
    private double TargetSpeed = 30;
    private double MaxSpeed = 30;

    // Motor
    private final TalonFX m_motor;

    // Motor Output
    private final VelocityVoltage m_output = new VelocityVoltage(TargetSpeed);

    // Toggle Boolean
    public boolean isOn = false;

    public Feed(int id) {
        kMotorID = id; 
        m_motor = new TalonFX(kMotorID, kMotorBus); 

        // Configure PID/feedforward gains for velocity control
        var slot0Configs = new Slot0Configs()
            .withKP(0.1)    // Proportional gain - adjust as needed
            .withKI(0.0)    // Integral gain
            .withKD(0.0)    // Derivative gain
            .withKS(0.0)    // Static friction feedforward
            .withKV(0.12);  // Velocity feedforward - tune this value

        var motorConfig = new TalonFXConfiguration()
            .withSlot0(slot0Configs);
        m_motor.getConfigurator().apply(motorConfig);
        m_motor.setNeutralMode(NeutralModeValue.Coast);

    }

    // Commands
    public Command intake() {
        return this.run(() -> {
            setMotorSpeed(TargetSpeed);
        });
    }

    public Command stop(){
        return this.runOnce(() -> {
            stopMotor();
        });
    }

    // Fuctions
    private void setMotorSpeed(double new_speed) {
        if(new_speed < 0.0){ 
            new_speed = 0.0;
        }

        if(new_speed > MaxSpeed){
            new_speed = MaxSpeed;
        }

        TargetSpeed = new_speed;

        m_output.Velocity = TargetSpeed; 
        m_motor.setControl(m_output);
        m_motor.setNeutralMode(NeutralModeValue.Brake);
        
        if (TargetSpeed == 0.0) {
            stopMotor();
        }
    }

    private void stopMotor() {
        m_motor.setControl(new StaticBrake());
    }
}

