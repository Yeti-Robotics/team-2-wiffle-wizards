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
    private final TalonFX roller1;
    private final TalonFX roller2;

    // Class containing the constants for the intake
    public static class IntakeConstants {
        public static final int intakeId = 9;
        public static final int roller1Id = 10;
        public static final int roller2Id = 11;
        public static final InvertedValue motorInversion = InvertedValue.Clockwise_Positive;
        public static final InvertedValue rollerInversion = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
        public static final double positionStatusFrame = 0.05;
        public static final double velocityStatusFrame = 0.01;
    }

    public IntakeSubsystem() {
        // Intake movement motor and roller motor initialization
        intakeMotor = new TalonFX(IntakeConstants.intakeId, "canivoreBus");
        roller1 = new TalonFX(IntakeConstants.roller1Id, "canivoreBus");
        roller2 = new TalonFX(IntakeConstants.roller2Id, "canivoreBus");

        // Intake motor configurator
        var intakeConfigurator = intakeMotor.getConfigurator();
        var configs = new TalonFXConfiguration();
        configs.MotorOutput.Inverted = IntakeConstants.motorInversion;
        configs.MotorOutput.NeutralMode = IntakeConstants.neutralMode;
        configs.FutureProofConfigs = Constants.TalonFXConstants.TALON_FUTURE_PROOF;
        intakeMotor.getRotorVelocity().waitForUpdate(IntakeConstants.velocityStatusFrame);
        intakeMotor.getRotorPosition().waitForUpdate(IntakeConstants.positionStatusFrame);
        intakeConfigurator.apply(configs);

        // Roller motor configurations
        var rollerConfigurator1 = roller1.getConfigurator();
        var rollerConfigurator2 = roller2.getConfigurator();
        var rollerConfigs = new TalonFXConfiguration();
        rollerConfigs.MotorOutput.Inverted = IntakeConstants.rollerInversion;
        rollerConfigs.MotorOutput.NeutralMode = IntakeConstants.neutralMode;
        rollerConfigs.FutureProofConfigs = Constants.TalonFXConstants.TALON_FUTURE_PROOF;
        roller1.getRotorVelocity().waitForUpdate(IntakeConstants.velocityStatusFrame);
        roller2.getRotorVelocity().waitForUpdate(IntakeConstants.velocityStatusFrame);
        rollerConfigurator1.apply(rollerConfigs);
        rollerConfigurator2.apply(rollerConfigs);
    }

    @Override
    public void periodic() {}

    // Sets intake speed
    private void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    // Set roller speed
    private void setRollerSpeed(double speed) { roller1.set(speed); roller2.set(speed); }

    // Stops intake movement
    private void stop() {
        intakeMotor.stopMotor();
    }

    // Command to spin the motor
    private Command moveIntake(double velocity, double timeout) {
        return startEnd(() -> setIntakeSpeed(velocity), this::stop).withTimeout(timeout);
    }

    public Command spinRollers(double speed, double timeout) {
        return startEnd(() -> setRollerSpeed(speed), this::stop).withTimeout(timeout);
    }

    // Command to raise the intake
    public Command raise(double velocity, double timeout) {
        if (velocity <= 0) {
            System.out.println("Velocity shouldn't be negative or 0 when attempting to manipulate intake.");
        }
        return moveIntake(Math.abs(velocity), timeout);
    }

    // Command to lower the intake
    public Command lower(double velocity, double timeout) {
        if (velocity <= 0) {
            System.out.println("Velocity shouldn't be negative or 0 when attempting to manipulate intake.");
        }
        return moveIntake(-Math.abs(velocity), timeout);
    }
}
