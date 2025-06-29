package frc.robot.subsystems;
import static frc.robot.PhotonvisonConstants.Vision.*;
import java.util.Optional;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OV9281Subsystem extends SubsystemBase {
    private final PhotonCamera camera; // webUI記得設定成一樣的 不然不會有數據
    private PhotonPipelineResult latestResult;
    private final EstimateConsumer estConsumer;
    private final PhotonPoseEstimator photonEstimator;
    private Matrix<N3, N1> curStdDevs;

    public OV9281Subsystem(EstimateConsumer estConsumer) {
        this.estConsumer = estConsumer;
        camera = new PhotonCamera(kCameraName);

        photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
        latestResult = camera.getLatestResult();

        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());

            visionEst.ifPresent(
                    est -> {
                        var estStdDevs = getEstimationStdDevs();
                        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    });
        }
        visionEst.ifPresent(est -> SmartDashboard.putString("Vision Pose", est.estimatedPose.toPose2d().toString()));
        SmartDashboard.putString("Vision StdDevs", getEstimationStdDevs().toString());
    }

    public PhotonPipelineResult getLatestResult() {
        return latestResult;
    }

    public Transform3d getBestTargetTransform() {
        if (latestResult.hasTargets()) {
            PhotonTrackedTarget target = latestResult.getBestTarget();
            return target.getBestCameraToTarget();
        }
        return null;
    }

    public int getBestTargetId() {
        if (latestResult.hasTargets()) {
            return latestResult.getBestTarget().getFiducialId();
        }
        return -1;
    }
    
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            curStdDevs = kSingleTagStdDevs;
        } else {
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist += tagPose.get().toPose2d().getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                curStdDevs = kSingleTagStdDevs;
            } else {
                avgDist /= numTags;
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdevs);
    }
}