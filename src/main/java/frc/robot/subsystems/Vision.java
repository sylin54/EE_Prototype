package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AprilTagFieldLayout;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase{
    PhotonCamera camera = new PhotonCamera(VisionConstants.CAMERA_NAME);

    VisionConsumer visionConsumer;

    public Vision(VisionConsumer visionConsumer) {
        this.visionConsumer = visionConsumer;
    }

    @Override
    public void periodic() {
        var result = camera.getLatestResult();

        boolean hasTargets = result.hasTargets();

        if(hasTargets) {
            PhotonTrackedTarget bestTarget = result.getBestTarget();

            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(bestTarget.getBestCameraToTarget(), AprilTagFieldLayout.getOffset(bestTarget.getFiducialId()).get(), VisionConstants.CAMERA_TO_ROBOT);

            visionConsumer.accept(robotPose.toPose2d(), result.getTimestampSeconds(), VecBuilder.fill(VisionConstants.linearStdDevBaseline, VisionConstants.linearStdDevBaseline, VisionConstants.angularStdDevBaseline));

        }
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs);
    }

}
