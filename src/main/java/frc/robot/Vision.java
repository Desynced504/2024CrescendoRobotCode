// package frc.robot;

// import java.util.Optional;
// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonUtils;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.targeting.PhotonPipelineResult;
// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation3d;
// // Shorten Constants Name for Camera Settings. Easier to read.
// import frc.robot.Constants.CameraConstants.*;

// public class Vision {

//     // The field from AprilTagFields will be different depending on the game.
//     private final AprilTagFieldLayout aprilTagFieldLayout = Constants.CameraConstants.aprilTagFieldLayout;
//     private final PhotonCamera harvesterCamera = new PhotonCamera(HarvesterCamera.name);
//     private final PhotonCamera shooterCamera = new PhotonCamera(ShooterCamera.name);

//     //Mounting Positions of Cameras
//     private final Transform3d harvesterRobotRelativeMountingPosition = new Transform3d
//         (
//             new Translation3d(HarvesterCamera.xDistanceForwardFromCenter
//                             , HarvesterCamera.yDistanceLeftFromCenter
//                             , HarvesterCamera.zDistanceUpFromFloor)
//             , new Rotation3d(HarvesterCamera.xRoll
//                             , HarvesterCamera.yPitchUp
//                             , HarvesterCamera.zYaw)
//         );
//     private final Transform3d shooterRobotRelativeMountingPosition = new Transform3d
//         (
//             // new Translation3d(), new Rotation3d()
//         new Translation3d(ShooterCamera.xDistanceForwardFromCenter
//                             , ShooterCamera.yDistanceLeftFromCenter
//                             , ShooterCamera.zDistanceUpFromFloor)
//             , new Rotation3d(ShooterCamera.xRoll
//                             , ShooterCamera.yPitchUp
//                             , ShooterCamera.zYaw)
//         );

//     // Construct PhotonPoseEstimator
//     private final PhotonPoseEstimator harvesterPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, harvesterCamera, harvesterRobotRelativeMountingPosition);
//     private final PhotonPoseEstimator shooterPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, shooterCamera, shooterRobotRelativeMountingPosition);

//     public Vision()
//     {
//         harvesterPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_LAST_POSE);
//         shooterPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_LAST_POSE);

//         setHarvesterPipelineIndex(1);
//         setShooterPipelineIndex(0);
//     }

//     public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFromHarvester(Pose2d prevEstimatedRobotPose) {
//         harvesterPhotonPoseEstimator.setLastPose(prevEstimatedRobotPose);
//         return harvesterPhotonPoseEstimator.update();
//     }

//     public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFromShooter(Pose2d prevEstimatedRobotPose) {
//         shooterPhotonPoseEstimator.setLastPose(prevEstimatedRobotPose);
//         // TODO: Combine harvesterPhotonEstimator with ShooterEstimator ONLY WHEN POSSIBLE!!!
//         return shooterPhotonPoseEstimator.update();
//     }

//     public EstimatedRobotPose getCombinedCameraPoseData(Pose2d lastEstimatedPosition)
//     {
//         Optional<EstimatedRobotPose> harvesterEstimatedPose, shooterEstimatedPose;

//         harvesterEstimatedPose = getEstimatedGlobalPoseFromHarvester(lastEstimatedPosition);
//         shooterEstimatedPose = getEstimatedGlobalPoseFromShooter(lastEstimatedPosition);

//         if (shooterEstimatedPose.isPresent())
//         {
//             return shooterEstimatedPose.get();
//         }

        
//         if (harvesterEstimatedPose.isPresent())
//         {
//             return harvesterEstimatedPose.get();
//         }
//         return null;


//     }


//     /**
//      * 0 - Note Detection
//      * 1 - AprilTag Detection
//      * 
//      * @param index - The Camera Pipeline Index to use during image processing (i.e. NoteDetection algorithm or AprilTag)
//      */
//     public void setHarvesterPipelineIndex(int index)
//     {
//         harvesterCamera.setPipelineIndex(index);
//     }

//     public void setShooterPipelineIndex(int index)
//     {
//         shooterCamera.setPipelineIndex(index);
//     }



//     public PhotonPipelineResult getHarvesterLatestResult()
//     {
//         return harvesterCamera.getLatestResult();
//     }

//     public PhotonPipelineResult getShooterLatestResult()
//     {
//         return shooterCamera.getLatestResult();
//     }


//     /**
//      * Calculates the angle the robot needs to intercept with a certain target (such as the speaker).
//      * Follows Path Planner Field Position and Robot Angle measurements.
//      * @param robotPose - Requires Position of Robot relative to field, (0,0) being bottom left of field (Blue Alliance Amp)
//      * @param targetPose - Requires Position of Target Object relative to field, (0,0) being bottom left of field (Blue Alliance Amp)
//      * @return The angle the robot must align with in order to intercept with the object in a straight line.
//      */
//     public static double getYawDegreesToTarget(Pose2d currentPose, Pose3d targetPose)
//     {
//         return PhotonUtils.getYawToPose(currentPose, targetPose.toPose2d()).getDegrees();
//     }


//     /**
//      * Calculates the pitch the robot needs to actuate shooter to reach a certain target (such as the speaker).
//      * Follows Path Planner Field Position and Robot Angle measurements.
//      * @param robotPose - Requires Position of Robot relative to field, (0,0) being bottom left of field (Blue Alliance Amp)
//      * @param targetPose - Requires Position of Target Object relative to field, (0,0) being bottom left of field (Blue Alliance Amp)
//      * @return The pitch the shooter must align with in order to intercept with the target in a straight line.
//      */
//     public static double getPitchDegreesToTarget(Pose2d currentPose, Pose3d targetPose)
//     {
//         double distanceToTarget = PhotonUtils.getDistanceToPose(currentPose, targetPose.toPose2d());
//         // (Target is most likely Speaker)
//         double targetHeight = targetPose.getZ();

//         try {
//             double tanh = (targetHeight / distanceToTarget); 
//             tanh = Math.toDegrees(Math.tanh(tanh));
//             if (tanh > Constants.ActuatorConstants.maxAngle)
//                 return Constants.ActuatorConstants.maxAngle;
//             else
//                 return tanh;
//         } catch (Exception e) {
//             System.out.println("Division By Zero?" + e);
//             return Constants.ActuatorConstants.minAngle; // Should be 90 but default angle will be 35 degrees for shooter for maximum safety and speed.
//         }
//     }
// }