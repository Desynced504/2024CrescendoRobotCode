// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import org.photonvision.EstimatedRobotPose;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Vision;

// public class CameraSubsystem extends SubsystemBase {

//   private static Vision cameras = new Vision();
//   private static EstimatedRobotPose estimatedRobotPose;
//   private static EstimatedRobotPose lastValidPose;

//   private static Pose2d m_robotPose = new Pose2d();

//   /*
//    * TODO: Create an object representing this class to track position variables, inlcude this subsystem in other constructors
//    * Update the global variable in other subsystems such as swerve.
//    */

//   /** Creates a new CameraSubsystem. */
//   public CameraSubsystem() {}


//  @Override
//   public void periodic() {}



//   public static EstimatedRobotPose getEstimatedRobotPose(Pose2d prevEstimatedRobotPose)
//   {
//     return cameras.getCombinedCameraPoseData(prevEstimatedRobotPose);
//   }

  


  
//   public static Pose2d getCameraSubsystemRobotPose() {
//     return m_robotPose;
//   }

//   public static void updateRobotPose(Pose2d robotPose)
//   {
//     m_robotPose = robotPose;
//   }


//   public static double getActuatorTargetAngle(Pose3d targetPose)
//   {
//     return Vision.getPitchDegreesToTarget(m_robotPose, targetPose);
//   }

//   public static double getSwerveTargetYaw(Pose3d targetPose)
//   {
//     return Vision.getYawDegreesToTarget(m_robotPose, targetPose);
//   }



// }
