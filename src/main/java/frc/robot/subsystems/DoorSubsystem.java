package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DoorSubsystem extends SubsystemBase {
    // Motor and sensor declarations
    private final TalonFX doorMotor;
    private final DigitalInput bottomLimitSwitch;
    private final DigitalInput topLimitSwitch;

    // Constants
    public static class DoorConstants{
        private static final int DOOR_MOTOR_ID = 6;
        private static final double MAX_SPEED = 1.0;
        private static final double DOOR_P = 0.1;
        private static final double DOOR_I = 0.0;
        private static final double DOOR_D = 0.0;
        private static final double DOOR_FF_A = 0.05;
        private static final double DOOR_FF_V = 8.0;
        private static final double PROFILE_V = 0.2;
        private static final double PROFILE_A = 0.5;

    }


    public DoorSubsystem() {
        // Initialize TalonFX motor with CAN ID
        doorMotor = new TalonFX(DoorConstants.DOOR_MOTOR_ID);
        bottomLimitSwitch = new DigitalInput(0);
        topLimitSwitch = new DigitalInput(1);

        // Configuration setup
        TalonFXConfiguration config = new TalonFXConfiguration();

        // PID and feedforward configuration
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.withKP(DoorConstants.DOOR_P).withKI(DoorConstants.DOOR_I).withKD(DoorConstants.DOOR_D)
                .withKA(DoorConstants.DOOR_FF_A).withKV(DoorConstants.DOOR_FF_V);
        config.Slot0 = slot0Configs;

        // Motion magic configuration
        config.MotionMagic.MotionMagicExpo_kV = DoorConstants.PROFILE_V;
        config.MotionMagic.MotionMagicExpo_kA = DoorConstants.PROFILE_A;

        // Current limits configuration
        config.CurrentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentThreshold(50)
                .withSupplyTimeThreshold(1)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(60);

        // Apply configurations to the motor
        doorMotor.getConfigurator().apply(config);

        // Set motor to brake mode
        doorMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    // Method to move the door up
    public void moveUp(double speed) {
        if (!topLimitSwitch.get()) {
            doorMotor.set(Math.abs(speed));
        } else {
            stop();
        }
    }

    // Method to move the door down
    public void moveDown(double speed) {
        if (!bottomLimitSwitch.get()) {
            doorMotor.set(-Math.abs(speed));
        } else {
            stop();
        }
    }

    // Stop the door motor
    public void stop() {
        doorMotor.stopMotor();
    }

    // Method to move the door to a specific position using Motion Magic
    public void setPosition(double targetPosition) {
        MotionMagicExpoVoltage motionMagicVoltage = new MotionMagicExpoVoltage(
                targetPosition, true, 0.0, 0, true, false, false);
        doorMotor.setControl(motionMagicVoltage);
    }

    // Check if the door is at the bottom
    public boolean isAtBottom() {
        return bottomLimitSwitch.get();
    }

    // Check if the door is at the top
    public boolean isAtTop() {
        return topLimitSwitch.get();
    }

    private Command moveDownAndStop(double speed){
        return startEnd(() -> moveDown(speed), this::stop);
    }

    private Command moveUpAndStop(double speed){
        return startEnd(() -> moveUp(speed), this::stop);
    }

    public Command goDown(double speed) {
        return runOnce(()-> moveDownAndStop(speed).until(this::isAtBottom));
    }

    public Command goUp(double speed) {
        return runOnce(()-> moveUpAndStop(speed).until(this::isAtTop));
    }

    @Override
    public void periodic() {
        // Reset encoder position if the door is at the bottom
        if (isAtBottom()) {
            doorMotor.setPosition(0.0);
        }
    }
}
