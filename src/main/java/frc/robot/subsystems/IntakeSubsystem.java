package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor;

    public static class IntakeConstants {
        public static final int krakenId = 9;
        public static final InvertedValue motorInversion = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
        public static final double positionStatusFrame = 0.05;
        public static final double velocityStatusFrame = 0.01;
    }

    public IntakeSubsystem() {
        intakeMotor = new TalonFX(IntakeConstants.krakenId, "canivoreBus");
        var intakeConfigurator = intakeMotor.getConfigurator();
        var configs = new TalonFXConfiguration();

        configs.MotorOutput.Inverted = IntakeConstants.motorInversion;
        configs.MotorOutput.NeutralMode = IntakeConstants.neutralMode;
        configs.FutureProofConfigs = Constants.TalonFXConstants.TALON_FUTURE_PROOF;
        intakeMotor.getRotorVelocity().waitForUpdate(IntakeConstants.velocityStatusFrame);
        intakeMotor.getRotorPosition().waitForUpdate(IntakeConstants.positionStatusFrame);
        intakeConfigurator.apply(configs);
    }

    @Override
    public void periodic() {}

    // Sets intake speed
    private void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    // Stops intake movement
    private void stop() {
        intakeMotor.stopMotor();
    }

    // Command to spin the motor
    private Command spin(double velocity, double timeout) {
        return startEnd(() -> setIntakeSpeed(velocity), this::stop).withTimeout(timeout);
    }

    // Command to raise the intake
    public Command raise(double velocity, double time) {
        if (velocity <= 0) {
            System.out.println("Velocity shouldn't be negative or 0 when attempting to manipulate intake.");
        }
        return spin(Math.abs(velocity), 1);
    }

    // Command to lower the intake
    public Command lower(double velocity, double time) {
        if (velocity <= 0) {
            System.out.println("Velocity shouldn't be negative or 0 when attempting to manipulate intake.");
        }
        return spin(-Math.abs(velocity), 1);
    }
}
