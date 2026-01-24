package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Shooter extends SubsystemBase {
    // Constants
    public static final CANBus kMotorBus = new CANBus("CANCAN");
    public static final int kMotorID = 21;
    public static final int kSpeed = 50;

    // Motor
    private final TalonFX m_motor = new TalonFX(kMotorID, kMotorBus);

    // Motor Output
    private final VelocityTorqueCurrentFOC m_output = new VelocityTorqueCurrentFOC(kSpeed);

    // Commands
    public Command shoot() {
        return this.runOnce(() -> {
            this.setMotorSpeed(kSpeed);
            new WaitCommand(0.5);
            this.stopMotor();
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
