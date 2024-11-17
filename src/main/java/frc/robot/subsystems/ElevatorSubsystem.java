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


public class ElevatorSubsystem extends SubsystemBase {
    // Motor and sensor declarations
    public final TalonFX elevatorMotor1;
    public final TalonFX elevatorMotor2;
    private final DigitalInput bottomLimitSwitch;
    private final DigitalInput topLimitSwitch;
    private final CANcoder elev1Encoder;
    private final CANcoder elev2Encoder;

    // Constants
    private static final int ELEVATOR_MOTOR_ID_1 = 4;
    private static final int ELEVATOR_MOTOR_ID_2 = 5;
    private static final double MAX_SPEED = 1.0;
    private static final double ELEVATOR_P = 0.1;
    private static final double ELEVATOR_I = 0.0;
    private static final double ELEVATOR_D = 0.0;
    private static final double ELEVATOR_FF_A = 0.05;
    private static final double ELEVATOR_FF_V = 8.0;
    private static final double PROFILE_V = 0.2;
    private static final double PROFILE_A = 0.5;

    public ElevatorSubsystem() {
        // Initialize TalonFX motor with CAN ID
        elevatorMotor1 = new TalonFX(ELEVATOR_MOTOR_ID_1, "rio");
        elevatorMotor2 = new TalonFX(ELEVATOR_MOTOR_ID_2, "rio");
        bottomLimitSwitch = new DigitalInput(6);
        topLimitSwitch = new DigitalInput(7);
        elev1Encoder = new CANcoder(4,"canbus");
        elev2Encoder = new CANcoder(5,"canbus");

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
        elevatorMotor1.getConfigurator().apply(config);
        elevatorMotor2.getConfigurator().apply(config);


        // Set motor to brake mode
        elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor2.setNeutralMode(NeutralModeValue.Brake);

        if (Utils.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(elevatorMotor1,elev1Encoder, 11,0.001);
            PhysicsSim.getInstance().addTalonFX(elevatorMotor2,elev2Encoder, 11,0.001);
        }
    }

    // Method to move the elevator up
    public void moveUp(double speed) {
        if (!topLimitSwitch.get()) {
            elevatorMotor1.set(Math.abs(speed));
            elevatorMotor2.set(Math.abs(speed));

        } else {
            stop();
        }
    }

    // Method to move the elevator down
    public void moveDown(double speed) {
        elevatorMotor1.set(-Math.abs(speed));
        elevatorMotor2.set(-Math.abs(speed));

    }

    // Stop the elevator motor
    public void stop() {
        elevatorMotor1.stopMotor();
        elevatorMotor2.stopMotor();

    }

    // Method to move the elevator to a specific position using Motion Magic
    public void setPosition(double targetPosition) {
        MotionMagicExpoVoltage motionMagicVoltage = new MotionMagicExpoVoltage(
                targetPosition, true, 0.0, 0, true, false, false);
        elevatorMotor1.setControl(motionMagicVoltage);
        elevatorMotor2.setControl(motionMagicVoltage);

    }

    // Check if the door is at the bottom
    public boolean isAtBottom() {return bottomLimitSwitch.get();}

    // Check if the door is at the top
    public boolean isAtTop() {return topLimitSwitch.get();}

    public Command moveDownAndStop(double speed){
        return startEnd(() -> moveDown(speed), this::stop).until(this::isAtBottom);
    }

    public Command moveUpAndStop(double speed){
        return startEnd(() -> moveUp(speed), this::stop).until(this::isAtTop);
    }


    @Override
    public void periodic() {
        // Reset encoder position if the elevator is at the bottom
        SmartDashboard.putData("elevator kraken 1", elevatorMotor1);
        SmartDashboard.putData("elevator kraken 2", elevatorMotor2);
        SmartDashboard.putData("elevator top limit", topLimitSwitch);
        SmartDashboard.putData("elevator bottom limit", bottomLimitSwitch);
    }
}
