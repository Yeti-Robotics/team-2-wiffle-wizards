package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sim.PhysicsSim;


public class DoorSubsystem extends SubsystemBase {
    // Motor and sensor declarations
    public final TalonFX doorMotor;
    private final DigitalInput bottomLimitSwitch;
    private final DigitalInput topLimitSwitch;
    public final CANcoder doorEncoder;

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
        doorMotor = new TalonFX(DoorConstants.DOOR_MOTOR_ID, "rio");
        bottomLimitSwitch = new DigitalInput(4);
        topLimitSwitch = new DigitalInput(5);
        doorEncoder = new CANcoder(6,"canbus");

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

        if (Utils.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(doorMotor,doorEncoder, 4,0.001);
            System.out.println("ADD DOOR MOTOR PLEASE");
        }


    }

    // Move door up if it hasn't reached top
    public void moveUp(double speed) {
        if (!topLimitSwitch.get()) {
            doorMotor.set(Math.abs(speed));
        } else {
            stop();
        }
    }

    // Move door down if it hasn't reached bottom
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

    // Move the door to a specific position using Motion Magic
    public void setPosition(double targetPosition) {
        MotionMagicExpoVoltage motionMagicVoltage = new MotionMagicExpoVoltage(
                targetPosition, true, 0.0, 0, true, false, false);
        doorMotor.setControl(motionMagicVoltage);
    }

    // Check if door is at the bottom
    public boolean isAtBottom() {
        return bottomLimitSwitch.get();
    }

    // Check if door is at the top
    public boolean isAtTop() {
        return topLimitSwitch.get();
    }

    public Command moveDownAndStop(double speed){
        return startEnd(() -> moveDown(speed), this::stop).until(this::isAtBottom);
    }

    public Command moveUpAndStop(double speed){
        return startEnd(() -> moveUp(speed), this::stop).until(this::isAtTop);
    }

    @Override
    public void periodic() {
        // Reset encoder position if the door is at the bottom
        SmartDashboard.putData("door kraken", doorMotor);
        SmartDashboard.putData("door top limit", topLimitSwitch);
        SmartDashboard.putData("door bottom limit", bottomLimitSwitch);

    }
}
