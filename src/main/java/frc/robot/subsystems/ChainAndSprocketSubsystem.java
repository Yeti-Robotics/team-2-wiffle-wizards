package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ChainAndSprocketSubsystem extends SubsystemBase {
    // Motor and sensor declarations
    private final TalonFX elevatorMotor;
    private final DigitalInput bottomLimitSwitch;
    private final DigitalInput topLimitSwitch;

    // Constants
    private static final int ELEVATOR_MOTOR_ID = 10;
    private static final double MAX_SPEED = 1.0;
    private static final double ELEVATOR_P = 0.1;
    private static final double ELEVATOR_I = 0.0;
    private static final double ELEVATOR_D = 0.0;
    private static final double ELEVATOR_FF_A = 0.05;
    private static final double ELEVATOR_FF_V = 8.0;
    private static final double PROFILE_V = 0.2;
    private static final double PROFILE_A = 0.5;

    public ChainAndSprocketSubsystem() {
        // Initialize TalonFX motor with CAN ID
        elevatorMotor = new TalonFX(ELEVATOR_MOTOR_ID);
        bottomLimitSwitch = new DigitalInput(0);
        topLimitSwitch = new DigitalInput(1);

        // Configuration setup
        TalonFXConfiguration config = new TalonFXConfiguration();

        // PID and feedforward configuration
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.withKP(ELEVATOR_P).withKI(ELEVATOR_I).withKD(ELEVATOR_D)
                .withKA(ELEVATOR_FF_A).withKV(ELEVATOR_FF_V);
        config.Slot0 = slot0Configs;

        // Motion magic configuration
        config.MotionMagic.MotionMagicExpo_kV = PROFILE_V;
        config.MotionMagic.MotionMagicExpo_kA = PROFILE_A;

        // Current limits configuration
        config.CurrentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentThreshold(50)
                .withSupplyTimeThreshold(1)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(60);

        // Apply configurations to the motor
        elevatorMotor.getConfigurator().apply(config);

        // Set motor to brake mode
        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    // Method to move the elevator up
    public void moveUp(double speed) {
        if (!topLimitSwitch.get()) {
            elevatorMotor.set(Math.abs(speed));
        } else {
            stop();
        }
    }

    // Method to move the elevator down
    public void moveDown(double speed) {
        if (!bottomLimitSwitch.get()) {
            elevatorMotor.set(-Math.abs(speed));
        } else {
            stop();
        }
    }

    // Stop the elevator motor
    public void stop() {
        elevatorMotor.stopMotor();
    }

    // Method to move the elevator to a specific position using Motion Magic
    public void setPosition(double targetPosition) {
        MotionMagicExpoVoltage motionMagicVoltage = new MotionMagicExpoVoltage(
                targetPosition, true, 0.0, 0, true, false, false);
        elevatorMotor.setControl(motionMagicVoltage);
    }

    // Check if the elevator is at the bottom
    public boolean isAtBottom() {
        return bottomLimitSwitch.get();
    }

    // Check if the elevator is at the top
    public boolean isAtTop() {
        return topLimitSwitch.get();
    }

    @Override
    public void periodic() {
        // Reset encoder position if the elevator is at the bottom
        if (isAtBottom()) {
            elevatorMotor.setPosition(0.0);
        }
    }
}