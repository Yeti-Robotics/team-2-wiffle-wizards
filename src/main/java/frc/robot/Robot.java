// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.sim.PhysicsSim;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;
    private static final double kMetersPerPulse = 0.01;
    private static final double kElevatorMinimumLength = 0.5;

    private final PWMSparkMax m_elevatorMotor = new PWMSparkMax(0);
    private final PWMSparkMax m_wristMotor = new PWMSparkMax(1);
    private final AnalogPotentiometer m_wristPot = new AnalogPotentiometer(1, 90);
    private final Encoder m_elevatorEncoder = new Encoder(0, 1);
    private final XboxController m_joystick = new XboxController(1);

    private MechanismLigament2d m_elevator;
    private MechanismLigament2d m_wrist;

    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
        m_elevatorEncoder.setDistancePerPulse(kMetersPerPulse);

        // the main mechanism object
        Mechanism2d mech = new Mechanism2d(3, 3);
        // the mechanism root node
        MechanismRoot2d root = mech.getRoot("climber", 2, 0);

        // MechanismLigament2d objects represent each "section"/"stage" of the mechanism, and are based
        // off the root node or another ligament object
        m_elevator = root.append(new MechanismLigament2d("elevator", kElevatorMinimumLength, 90));
        m_wrist =
                m_elevator.append(
                        new MechanismLigament2d("wrist", 0.5, 90, 6, new Color8Bit(Color.kPurple)));

        // post the mechanism to the dashboard
        SmartDashboard.putData("Mech2d", mech);
    }


    /**
     * This method is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        m_elevator.setLength(kElevatorMinimumLength + m_elevatorEncoder.getDistance());
        m_wrist.setAngle(m_wristPot.get());
    }


    /**
     * This method is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
    }


    @Override
    public void disabledPeriodic() {
    }


    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        //autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }


    /**
     * This method is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }


    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }


    /**
     * This method is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        m_elevatorMotor.set(m_joystick.getRawAxis(0));
        m_wristMotor.set(m_joystick.getRawAxis(1));
    }


    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }


    /**
     * This method is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }


    /**
     * This method is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
        robotContainer.simulationInit();

    }


    /**
     * This method is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
        if (robotContainer != null) {
            robotContainer.simulationPeriodic();
        }
        PhysicsSim.getInstance().run();

    }
}
