package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

public class RobotPose {
    Pose3d drivePose = new Pose3d();
    Pose3d intakePose = new Pose3d();

    StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
            .getStructTopic("DavePose", Pose3d.struct).publish();
    StructArrayPublisher<Pose3d> arrayPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("DavePoseArray", Pose3d.struct).publish();

    public void periodic() {
        publisher.set(drivePose);
        arrayPublisher.set(new Pose3d[] {drivePose, intakePose});
    }
}
