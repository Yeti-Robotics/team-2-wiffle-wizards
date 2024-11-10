// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.generated.TunerConstants;
import frc.robot.util.controllerUtils.ControllerContainer;


public class RobotContainer {

    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final DoorSubsystem door = new DoorSubsystem();
    public final CommandXboxController joystick = new CommandXboxController(1); // My joystick

    final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(CommandSwerveDrivetrain.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
            .withRotationalDeadband(CommandSwerveDrivetrain.MaFxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric

    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    //public ControllerContainer controllerContainer = new ControllerContainer();
    //public SendableChooser<AutoConstants.AutoMode> autoChooser;
    private boolean autoNeedsRebuild = true;
    private Command auto;

    private final RobotCommands robotCommands = new RobotCommands(
            intake,  drivetrain, elevator, door
    );

    public RobotContainer() {
        NamedCommands.registerCommand("dump", Commands.sequence(

        ));

        var field = new Field2d();
        SmartDashboard.putData("Field", field);


        PathPlannerLogging.setLogCurrentPoseCallback(field::setRobotPose);

        PathPlannerLogging.setLogTargetPoseCallback(pose ->
                field.getObject("target pose").setPose(pose)
        );

        PathPlannerLogging.setLogActivePathCallback(poses ->
                field.getObject("path").setPoses(poses)
        );


        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        configureBindings();

        //buildAutoChooser();
        //rebuildAutoIfNecessary();

        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        SmartDashboard.putNumber("shooterstate-position", 0.5);
        SmartDashboard.putBoolean("elev mag switch", elevator.isAtBottom());
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () ->
                                drive
                                        // +X in velocity = forward, -Y in joystick = forward
                                        .withVelocityX(-joystick.getLeftY() * TunerConstants.kSpeedAt12VoltsMps)
                                        // +Y in velocity = left, -X in joystick = left
                                        .withVelocityY(-joystick.getLeftX() * TunerConstants.kSpeedAt12VoltsMps)
                                        // +rotational rate = counterclockwise (left), -X in joystick = left
                                        .withRotationalRate(-joystick.getRightX() * CommandSwerveDrivetrain.MaFxAngularRate)
                ));


        // Reset the field-centric heading
        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

        // Lower intake to begin collecting electrolytes
        joystick.leftBumper().onTrue(intake.lower(0.7,1));

        // Collect electrolytes in
        joystick.rightBumper().whileTrue(intake.spinRollers(0.7,2));

        // Raise lift based on speed of trigger
        joystick.leftTrigger().whileTrue(elevator.goUp(0.7));

        // Move door based on speed of trigger
        joystick.rightTrigger().whileTrue(door.goUp(joystick.getRightTriggerAxis()));

        // Pull intake back in
        joystick.a().onTrue(intake.raise(0.7,1));

        //Close door
        joystick.x().onTrue(door.goDown(0.6));

        //
        
        // Swerve lock
        joystick.y().whileTrue(drivetrain
                .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));    }
    /*
    public void buildAutoChooser() {
        var namedCommands = new AutoNamedCommands(intake, shooter, pivot, arm, robotCommands);
        namedCommands.registerCommands();

        autoChooser = new SendableChooser<>();

        for (var autoMode : AutoConstants.AutoMode.values()) {
            autoChooser.addOption(autoMode.name, autoMode);
        }
        // note: setDefaultOption overwrites the name in the map, so we won't have duplicate options
        autoChooser.setDefaultOption(AutoConstants.AutoMode.BUMP_ONLY.name, AutoConstants.AutoMode.BUMP_ONLY);

        autoChooser.onChange(obj -> autoNeedsRebuild = true);

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public void rebuildAutoIfNecessary() {
        if (autoNeedsRebuild) {
            // why is there a timeout here? can't we use the FMS/practice mode timeout?
            auto = AutoBuilder.buildAuto(autoChooser.getSelected().name).withTimeout(15);
            System.out.println("AUTO NAME: " + auto);
            autoNeedsRebuild = false;
        }
    }

    public Command getAutonomousCommand() {
        return auto;
    }
    */
}
