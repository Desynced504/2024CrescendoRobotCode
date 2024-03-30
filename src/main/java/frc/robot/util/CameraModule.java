package frc.robot.util;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;

/** Add your docs here. */
public class CameraModule
{

    // Field April Tag Layout
    private final AprilTagFieldLayout aprilTagFieldLayout = Constants.CameraConstants.aprilTagFieldLayout;

    // Photon Camera
    private final PhotonCamera camera;

    //Mounting Positions of Cameras
    private final Transform3d robotRelativeMountingPosition;

    // PhotonPoseEstimator
    private final PhotonPoseEstimator photonPoseEstimator;

    // Standard Deviations for Camera Measurements (Larger values means less trusted)
    // TODO: Measure n1 and n2 standard devs.
	private static final Vector<N3> STANDARD_DEVS = VecBuilder.fill(0.2, 0.2, Double.POSITIVE_INFINITY);

    private Optional<EstimatedRobotPose> latestPose = Optional.empty();
    private PhotonPipelineResult latestResult = null;
    private boolean latestResultIsValid = false;
    private double lastValidTimestampSeconds = 0;
	private Pose2d lastFieldPose = new Pose2d(-1, -1, new Rotation2d());


    public CameraModule(PhotonCamera camera, Transform3d robotToCameraTransformation, int startingPipeline)
    {
        this.camera = camera;
        this.robotRelativeMountingPosition = robotToCameraTransformation;
        this.photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotRelativeMountingPosition);

        setPipelineIndex(startingPipeline);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_LAST_POSE);
        photonPoseEstimator.setLastPose(lastFieldPose);
    }


    /**
     * Pipelines:
     * 0 - Note Detection,
     * 1 - AprilTag Detection
     * 
     * @param index - The Camera Pipeline Index to use during image processing (i.e. NoteDetection algorithm or AprilTag)
     */
    public void setPipelineIndex(int index)
    {
        camera.setPipelineIndex(index);
    }
    

    public PhotonPipelineResult getLatestResult()
    {
        return this.camera.getLatestResult();
    }

    public PhotonCamera getCamera() {
        return camera;
    }


	private boolean isCameraResultValid(PhotonPipelineResult result) {
		for (var target : result.targets) {
			if (target.getPoseAmbiguity() > 0.2) {
				return false;
			}
		}
		return true;
	}

    public void update(SwerveDrivePoseEstimator swerveDrive)
    {
        System.out.println("Updating april tags. . ." + System.currentTimeMillis());
        latestResult = camera.getLatestResult();
		latestResultIsValid = isCameraResultValid(latestResult);
		lastValidTimestampSeconds = latestResult.getTimestampSeconds();

        if (!latestResultIsValid) {
			return;
		}
		latestPose = photonPoseEstimator.update(latestResult);
		if (latestPose.isPresent()) {
            System.out.println("Updating Pose. . .");
			lastValidTimestampSeconds = latestPose.get().timestampSeconds;
			lastFieldPose = latestPose.get().estimatedPose.toPose2d();
			swerveDrive.addVisionMeasurement(lastFieldPose, lastValidTimestampSeconds, STANDARD_DEVS);
			var estimatedPose = swerveDrive.getEstimatedPosition();
			photonPoseEstimator.setLastPose(estimatedPose);
		}
    }


	/**
	 * Returns null if there is no known robot pose.
	 *
	 * @return The calculated robot pose in meters.
	 */
	public Pose3d getRobotPose() {
		if (latestPose.isPresent()) {
			return latestPose.get().estimatedPose;
		}
		return null;
	}

}