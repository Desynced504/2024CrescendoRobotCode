// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.EnumSet;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.CameraConstants.HarvesterCamera;
import frc.robot.Constants.CameraConstants.ShooterCamera;

/** Add your docs here. */
public class PhotonVisionWrapper
{



    private final CameraModule harvesterCamera = new CameraModule(
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


    private final CameraModule shooterCamera = new CameraModule(
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

    private final SwerveDrivePoseEstimator m_swerveDrive;

    public PhotonVisionWrapper(SwerveDrivePoseEstimator swerveDriveOdometry)
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
        return m_swerveDrive.getEstimatedPosition();
    }
}