package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    // Constants
    public static final CANBus kMotorBus = new CANBus("CANCAN");
    public static final int kMotorID = 21;
    public static final int kSpeed = -65;

    // Motor
    private final TalonFX m_motor = new TalonFX(kMotorID, kMotorBus);

    // Motor Output
    private final VelocityTorqueCurrentFOC m_output = new VelocityTorqueCurrentFOC(kSpeed);

    // Toggle Boolean
    public boolean isOn = false;

    public Shooter() {
        // Configure PID/feedforward gains for velocity control
        var slot0Configs = new Slot0Configs()
            .withKP(5.0)    // Proportional gain - adjust as needed
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
    public Command toggleShooting() {
        return this.runOnce(() -> {
            isOn = !isOn;

            if (isOn) {
                setMotorSpeed(kSpeed);
            } else {
                stopMotor();
            }
        });
    }

    // Fuctions
    private void setMotorSpeed(double speed) {
        m_output.Velocity = speed;
        m_motor.setControl(m_output);
        m_motor.setNeutralMode(NeutralModeValue.Brake);

        if (speed == 0.0) {
            stopMotor();
        }
    }

    private void stopMotor() {
        m_motor.setControl(new StaticBrake());
    }
}
