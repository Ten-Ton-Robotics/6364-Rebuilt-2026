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
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Shooter extends SubsystemBase {
    // Constants
    public static final CANBus kMotorBus = CANBus.roboRIO();
    public static final int kMotorID = 21;
    public static final int kSpeed = 80;

    // Controller gains
    public static final double kKP = 0.35;
    public static final double kKI = 0;
    public static final double kKD = 0;
    public static final double kKS = 0;
    public static final double kKV = 0;
    public static final double kKA = 0;

    // Drive Ratio
    public static final double kRatio = 1;

    // Motor
    private final TalonFX m_motor = new TalonFX(kMotorID, kMotorBus);

    // Motor Output
    private final VelocityTorqueCurrentFOC m_output = new VelocityTorqueCurrentFOC(kSpeed);

    public Shooter() {
        super();

        // configure motor
        final TalonFXConfiguration config = new TalonFXConfiguration();

        // set controller gains
        config.Slot0 = new Slot0Configs().withKP(kKP).withKI(kKI).withKD(kKD).withKS(kKS).withKV(kKV).withKA(kKA);

        // set ratio
        config.Feedback.SensorToMechanismRatio = kRatio;

        // apply configuration
        m_motor.getConfigurator().apply(config);
    }

    // Commands
    public Command shoot() {
        return this.runOnce(() -> {
            this.setMotorSpeed(kSpeed);
            new WaitCommand(5);
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
