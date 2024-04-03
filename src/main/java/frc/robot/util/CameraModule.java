package frc.robot.util;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.Matrix;

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
	// private final Vector<N3> STANDARD_DEVS = VecBuilder.fill(1, 1, Double.POSITIVE_INFINITY);
    // private Matrix<N3, N1>  visionmeasurementStdDevsMatrix = VecBuilder.fill(1, 1, 100);

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

    public Optional<Double> getYawToTargetAprilTag(int aprilTagTargetID)
    {

        for (PhotonTrackedTarget photonTrackedTarget : camera.getLatestResult().targets)
        {
            if (photonTrackedTarget.getFiducialId() == aprilTagTargetID)
            {
                Optional<Double> yaw = Optional.of(photonTrackedTarget.getYaw());
                return yaw;
            }
        }
        return Optional.empty();
    }


	private boolean isCameraResultValid(PhotonPipelineResult result) {
		for (var target : result.targets) {
			if (target.getPoseAmbiguity() > 0.2) {
				return false;
			}
		}
		return true;
	}


    public void update(SwerveSubsystem swerveSubsystem)
    {
        switch (camera.getPipelineIndex()) {
            case 0:
                updatePose(swerveSubsystem);
                break;
            case 1:
                updateNoteDetection();
            default:
                System.out.println("Cannot Find Camera Pipeline!");
                break;
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


    public void updatePose(SwerveSubsystem swerveSubsystem)
    {
        SwerveDrivePoseEstimator swerveDrive = swerveSubsystem.getSwerveDriveOdometry();
        // System.out.println("Updating april tags. . ." + System.currentTimeMillis());
        latestResult = camera.getLatestResult();
		latestResultIsValid = isCameraResultValid(latestResult);
		lastValidTimestampSeconds = latestResult.getTimestampSeconds();

        if (!latestResultIsValid) {
			return;
		}
		latestPose = photonPoseEstimator.update(latestResult);
		if (latestPose.isPresent()) {
            // System.out.println("Updating Pose. . ." + latestPose.get().estimatedPose);
			lastValidTimestampSeconds = latestPose.get().timestampSeconds;
			lastFieldPose = latestPose.get().estimatedPose.toPose2d();
            
            try {
                // swerveSubsystem.updateState();
                // System.out.println("-------------------Testing-----------\n" + System.currentTimeMillis());
                // swerveSubsystem.addVisionMeasurement(lastFieldPose, lastValidTimestampSeconds);
                swerveSubsystem.addVisionMeasurement(lastFieldPose, lastValidTimestampSeconds, VecBuilder.fill(0.9, 0.9, 0.9));

                // swerveSubsystem.setGyroOffset(swerveSubsystem.getPigeon2().getRotation2d().plus(new Rotation2d(Math.PI)));
                // swerveDrive.resetPosition(
                //     swerveSubsystem.getPigeon2().getRotation2d()
                //     sample.get().wheelPositions,
                //     sample.get().poseMeters.exp(scaledTwist));



            } catch (Exception e) {
                // TODO: handle exception
                System.out.println("==================================");
                System.out.println("==================================");
                System.out.println("HUGE ERROR WITH CAMERA MODLUES AND CRASHING AND CONCURRENT MODIFICATION AND SWERVE POSE UPDATES OUT OF SYNC");
                e.printStackTrace();
                System.out.println("==================================");
                System.out.println("==================================");
            }

            // swerveDrive.addVisionMeasurement(lastFieldPose, lastValidTimestampSeconds, visionmeasurementStdDevsMatrix);
			var estimatedPose = swerveDrive.getEstimatedPosition();
			photonPoseEstimator.setLastPose(estimatedPose);
		}
    }

    public void updateNoteDetection()
    {
        // TODO: Tune Best Target Settings
        latestResult = camera.getLatestResult();
        latestResultIsValid = latestResult.hasTargets();
        if (latestResultIsValid)
        {
            double targetHeight, targetPitchRadians;
            System.out.println(latestResult);
            // PhotonUtils.calculateDistanceToTargetMeters(robotRelativeMountingPosition.getZ(), 0, robotRelativeMountingPosition.getRotation().getY(), 0);
        }
    }

}








// Concurrent Modification Error found
/*
    public void updatePose(SwerveSubsystem swerveSubsystem)
    {
        SwerveDrivePoseEstimator swerveDrive = swerveSubsystem.getSwerveDriveOdometry();
        // System.out.println("Updating april tags. . ." + System.currentTimeMillis());
        latestResult = camera.getLatestResult();
		latestResultIsValid = isCameraResultValid(latestResult);
		lastValidTimestampSeconds = latestResult.getTimestampSeconds();

        if (!latestResultIsValid) {
			return;
		}
		latestPose = photonPoseEstimator.update(latestResult);
		if (latestPose.isPresent()) {
            // System.out.println("Updating Pose. . ." + latestPose.get().estimatedPose);
			lastValidTimestampSeconds = latestPose.get().timestampSeconds;
			lastFieldPose = latestPose.get().estimatedPose.toPose2d();
            


            // Error lies in these 2 lines of code, when trying to update pose,
            // They both access the odometry and cause overrun error.


            swerveSubsystem.updateState();
            swerveDrive.addVisionMeasurement(lastFieldPose, lastValidTimestampSeconds);
            
            
            
            
            // swerveDrive.addVisionMeasurement(lastFieldPose, lastValidTimestampSeconds, visionmeasurementStdDevsMatrix);
			var estimatedPose = swerveDrive.getEstimatedPosition();
			photonPoseEstimator.setLastPose(estimatedPose);
		}
    }
*/