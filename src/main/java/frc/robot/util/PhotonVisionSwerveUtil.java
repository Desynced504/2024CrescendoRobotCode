// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.EnumSet;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.Constants.CameraConstants.HarvesterCamera;
import frc.robot.Constants.CameraConstants.ShooterCamera;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class PhotonVisionSwerveUtil
{

    private static final CameraModule harvesterCamera = new CameraModule(
        new PhotonCamera(HarvesterCamera.name)
        , new Transform3d(
            new Translation3d(HarvesterCamera.xDistanceForwardFromCenter
                , HarvesterCamera.yDistanceLeftFromCenter
                , HarvesterCamera.zDistanceUpFromFloor)
            , new Rotation3d(HarvesterCamera.xRoll
                , HarvesterCamera.yPitchUp
                , HarvesterCamera.zYaw)
            )
        , 0
        );


    private static final CameraModule shooterCamera = new CameraModule(
        new PhotonCamera(ShooterCamera.name)
        , new Transform3d(
            new Translation3d(ShooterCamera.xDistanceForwardFromCenter
                , ShooterCamera.yDistanceLeftFromCenter
                , ShooterCamera.zDistanceUpFromFloor)
            , new Rotation3d(ShooterCamera.xRoll
                , ShooterCamera.yPitchUp
                , ShooterCamera.zYaw)
            )
        , 0
        );


    private final SwerveSubsystem m_swerveDrive;

    public PhotonVisionSwerveUtil(SwerveSubsystem swerveDriveOdometry)
    {
        this.m_swerveDrive = swerveDriveOdometry;

		var networkTables = NetworkTableInstance.getDefault();

        networkTables.addListener(
				networkTables
						.getTable("photonvision")
						.getSubTable(harvesterCamera.getCamera().getName())
						.getEntry("rawBytes"),
				EnumSet.of(NetworkTableEvent.Kind.kValueAll),
				event -> harvesterCamera.update(m_swerveDrive));

        // TODO: AFTER TESTING CAMERAS INDIVIDUALLY, ENSURE THAT ADD_REQUIREMENTS DOES NOT OVERRIDE ACCESS TO SWERVE!!!

        networkTables.addListener(
				networkTables
						.getTable("photonvision")
						.getSubTable(shooterCamera.getCamera().getName())
						.getEntry("rawBytes"),
				EnumSet.of(NetworkTableEvent.Kind.kValueAll),
				event -> shooterCamera.update(m_swerveDrive));
    }

    public Pose2d getRobotPose()
    {
        return m_swerveDrive.getSwerveDriveOdometry().getEstimatedPosition();
    }

    public CameraModule getShooterCamera() {
        return shooterCamera;
    }

    public CameraModule getHarvesterCamera() {
        return harvesterCamera;
    }


    /**
     * Calculates the angle the robot needs to intercept with a certain target (such as the speaker).
     * Follows Path Planner Field Position and Robot Angle measurements.
     * @param robotPose - Requires Position of Robot relative to field, (0,0) being bottom left of field (Blue Alliance Amp)
     * @param targetPose - Requires Position of Target Object relative to field, (0,0) being bottom left of field (Blue Alliance Amp)
     * @return The angle the robot must align with in order to intercept with the object in a straight line.
     */
    public static Rotation2d getRotationToTarget(Pose2d currentPose, Pose3d targetPose)
    {
        /*
         * getYawToPose returns a Rotation2d with the distance to target yaw.
         * The robot's current yaw is added to make the heading field-centric for swerve heading controllers.
         */
        return (PhotonUtils.getYawToPose(currentPose, targetPose.toPose2d()).plus(currentPose.getRotation()));
    }


    /**
     * Calculates the pitch the robot needs to actuate shooter to reach a certain target (such as the speaker).
     * Follows Path Planner Field Position and Robot Angle measurements.
     * @param robotPose - Requires Position of Robot relative to field, (0,0) being bottom left of field (Blue Alliance Amp)
     * @param targetPose - Requires Position of Target Object relative to field, (0,0) being bottom left of field (Blue Alliance Amp)
     * @return The pitch the shooter must align with in order to intercept with the target in a straight line.
     */
    public static double getActuatorPitchDegreesToTarget(Pose2d currentPose, Pose3d targetPose)
    {
        // Adding 0.254 Meters to distance calculation as the shooter is 10 inches from the center and this is where the angle is from.
        double distanceToTarget = 0.254 + PhotonUtils.getDistanceToPose(currentPose, targetPose.toPose2d());
        // (Target is most likely Speaker)
        double targetHeight = targetPose.getZ();

        try {
            double tanh = (targetHeight / distanceToTarget); 
            tanh = Math.toDegrees(Math.tanh(tanh));
            if (tanh > Constants.ActuatorConstants.maxAngle)
                return Constants.ActuatorConstants.maxAngle;
            else
                return tanh;
        } catch (Exception e) {
            System.out.println("Division By Zero?" + e);
            return Constants.ActuatorConstants.minAngle; // Should be 90 but default angle will be 35 degrees for shooter for maximum safety and speed.
        }
    }

}


/*
    Overrun Error:
    
    CommandScheduler loop overrun
    at java.base/java.lang.Thread.run(Unknown Source)
    at edu.wpi.first.networktables.NetworkTableInstance$ListenerStorage.lambda$startThread$0(NetworkTableInstance.java:894)
    at frc.robot.util.PhotonVisionSwerveUtil.lambda$new$1(PhotonVisionSwerveUtil.java:80)
    at frc.robot.util.CameraModule.update(CameraModule.java:102)
    at frc.robot.util.CameraModule.updatePose(CameraModule.java:141)
    at edu.wpi.first.math.estimator.PoseEstimator.addVisionMeasurement(PoseEstimator.java:191)
    at java.base/java.util.TreeMap$NavigableSubMap$SubMapEntryIterator.next(Unknown Source)
    at java.base/java.util.TreeMap$NavigableSubMap$SubMapEntryIterator.next(Unknown Source)
    at java.base/java.util.TreeMap$NavigableSubMap$SubMapIterator.nextEntry(Unknown Source)

    java.util.ConcurrentModificationException
    Unhandled exception during listener callback: java.util.ConcurrentModificationException
    Warning at edu.wpi.first.wpilibj.IterativeRobotBase.printLoopOverrunMessage(IterativeRobotBase.java:412): Loop time of 0.02s overrun 

 */


/*
    Concurrent Modification Exception:

    Unhandled exception during listener callback: java.util.ConcurrentModificationException 

 	at java.base/java.lang.Thread.run(Unknown Source)
    at edu.wpi.first.networktables.NetworkTableInstance$ListenerStorage.lambda$startThread$0(NetworkTableInstance.java:894)
    at frc.robot.util.PhotonVisionSwerveUtil.lambda$new$1(PhotonVisionSwerveUtil.java:80)
    at frc.robot.util.CameraModule.update(CameraModule.java:102)
    at frc.robot.util.CameraModule.updatePose(CameraModule.java:141)
    at edu.wpi.first.math.estimator.PoseEstimator.addVisionMeasurement(PoseEstimator.java:191)
    at java.base/java.util.TreeMap$NavigableSubMap$SubMapEntryIterator.next(Unknown Source)
    at java.base/java.util.TreeMap$NavigableSubMap$SubMapEntryIterator.next(Unknown Source)
    at java.base/java.util.TreeMap$NavigableSubMap$SubMapIterator.nextEntry(Unknown Source)
    java.util.ConcurrentModificationException 
*/