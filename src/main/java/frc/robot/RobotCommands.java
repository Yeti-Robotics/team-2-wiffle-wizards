package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.RobotDataPublisher;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.controllerUtils.MultiButton;

public class RobotCommands {

    public RobotCommands(IntakeSubsystem intake, CommandSwerveDrivetrain commandSwerveDrivetrain, ElevatorSubsystem elevator, DoorSubsystem door) {
    }

    /**
     * Requires
     * Sets shooter state in preparation for shooting
     *
     * @return {@code Command} instance
     */

}