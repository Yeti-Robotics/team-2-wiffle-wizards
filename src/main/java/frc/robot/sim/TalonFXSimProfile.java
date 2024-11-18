package frc.robot.sim;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.sim.PhysicsSim.SimProfile;

/**
 * Holds information about a simulated TalonFX.
 */
class TalonFXSimProfile extends SimProfile {
    private static final double kMotorResistance = 0.002; // Assume 2mOhm resistance for voltage drop calculation
    private final TalonFX _talonFX;
    private final CANcoder _canCoder;
    private final double _gearRatio;

    private final DCMotorSim _motorSim;

    /**
     * Creates a new simulation profile for a TalonFX device.
     *
     * @param talonFX
     *                        The TalonFX device
     * @param canCoder
     *                        The CANcoder associated with the TalonFX
     * @param gearRatio
     *                        The gear ratio from the TalonFX to the mechanism
     * @param rotorInertia
     *                        Rotational Inertia of the mechanism at the rotor
     */
    public TalonFXSimProfile(final TalonFX talonFX, final CANcoder canCoder, final double gearRatio, final double rotorInertia) {
        this._talonFX = talonFX;
        this._canCoder = canCoder;
        this._gearRatio = gearRatio;
        this._motorSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), gearRatio, rotorInertia);
    }

    /**
     * Runs the simulation profile.
     *
     * This uses very rudimentary physics simulation and exists to allow users to
     * test features of our products in simulation using our examples out of the
     * box. Users may modify this to utilize more accurate physics simulation.
     */
    public void run() {
        // DEVICE SPEED SIMULATION
        _motorSim.setInputVoltage(_talonFX.getSimState().getMotorVoltage());

        _motorSim.update(getPeriod());

        // SET SIM PHYSICS INPUTS
        final double position_rot = _motorSim.getAngularPositionRotations() * _gearRatio;
        final double velocity_rps = Units.radiansToRotations(_motorSim.getAngularVelocityRadPerSec()) * _gearRatio;

        _talonFX.getSimState().setRawRotorPosition(position_rot);
        _talonFX.getSimState().setRotorVelocity(velocity_rps);

        _talonFX.getSimState().setSupplyVoltage(12 - _talonFX.getSimState().getSupplyCurrent() * kMotorResistance);

        _canCoder.getSimState().setRawPosition(position_rot / _gearRatio);
        _canCoder.getSimState().setVelocity(velocity_rps / _gearRatio);
    }
}