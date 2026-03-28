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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Pivot extends SubsystemBase {
    // Constants
    private static final CANBus kMotorBus = new CANBus("CANCAN");

    private static final int kMotorID = 18; //Get motor ID from TunerX put that one here
    private static double TargetSpeed = 30; 

    // Motor
    private final TalonFX m_motor = new TalonFX(kMotorID, kMotorBus);

    // Motor Output
    private final VelocityVoltage m_output = new VelocityVoltage(TargetSpeed);

    // Toggle Boolean
    public boolean isOn = false;

    public Pivot() {
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
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive) //Fowards pushes the intake down
            );
        m_motor.getConfigurator().apply(motorConfig);
        m_motor.setNeutralMode(NeutralModeValue.Coast);

    }

    // Commands
    public Command pivotDown() {
        return this.run(() -> {
            setMotorSpeed(TargetSpeed);
        });
    }

    public Command pivotUp() {
        return this.run(() -> {
            setMotorSpeed(-TargetSpeed);
        });
    }

    public Command stop(){
        return this.runOnce(() -> {
            stopMotor();
        });
    }

    public Command pivotChooChoo() {
        return new SequentialCommandGroup(
            pivotUp(),
            new WaitCommand(0.5),
            stop(),
            new WaitCommand(0.3),
            pivotDown(),
            new WaitCommand(0.5),
            stop()
        );
    }

    // Functions
    private void setMotorSpeed(double new_speed) {
        m_output.Velocity = new_speed; 
        m_motor.setControl(m_output);
        m_motor.setNeutralMode(NeutralModeValue.Coast);
        
        if (new_speed == 0.0) {
            stop();
        }
    }

    private void stopMotor() {
        m_motor.setControl(new StaticBrake());
    }
}

