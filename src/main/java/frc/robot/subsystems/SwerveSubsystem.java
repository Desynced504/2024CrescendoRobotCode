package frc.robot.subsystems;

import java.util.Timer;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class SwerveSubsystem extends SwerveDrivetrain implements Subsystem {

    private Field2d cameraFieldPoseEstimate = new Field2d(); // SmartDashboard Representation of Robot Position
    private Field2d swerveFieldPoseEstimate = new Field2d(); // SmartDashboard Representation of Robot Position
    private Field2d fusedFieldPoseEstimate = new Field2d(); // SmartDashboard Representation of Robot Position
    private double timer;

    /** Swerve Controller From Chassis Speeds */
    private final SwerveRequest.ApplyChassisSpeeds chassisSpeedsSwerveRequest = new SwerveRequest.ApplyChassisSpeeds();
    /** Swerve Controller From FieldCentric Driving */
    private final SwerveRequest.FieldCentric FieldCentricSwerveRequest = new SwerveRequest.FieldCentric()
                .withDeadband(Constants.SwerveDriveConstants.MAX_TELE_DRIVE_SPEED * 0.1).withRotationalDeadband(Constants.SwerveDriveConstants.MAX_ANGULAR_RATE * 0.1) // Add a 10% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    /** Swerve Controller From FieldCentric Facing a Target Angle While Driving */
    private final SwerveRequest.FieldCentricFacingAngle FieldCentricFacingAngleSwerveRequest = new SwerveRequest.FieldCentricFacingAngle()
                .withDeadband(Constants.SwerveDriveConstants.MAX_TELE_DRIVE_SPEED * 0.1);
    /** Swerve Controller From FieldCentric Facing a Target Angle While Driving */
    private final SwerveRequest.SwerveDriveBrake xBrakeSwerveRequest = new SwerveRequest.SwerveDriveBrake();

    // TODO: Make sure this works, should prevent motors from drawing too much current / spiking too much. Makes driving impossible
    private final CurrentLimitsConfigs driveMotorSettings = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(0)
        .withStatorCurrentLimitEnable(false)
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(80);

    private final CurrentLimitsConfigs steerMotorSettings = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(0)
        .withStatorCurrentLimitEnable(false)
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(20);


    public SwerveSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        this(driveTrainConstants, 0, modules);
    }
    
    public SwerveSubsystem(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);

        for (SwerveModule module : this.Modules) {
            module.getDriveMotor().getConfigurator().apply(driveMotorSettings);
            module.getSteerMotor().getConfigurator().apply(steerMotorSettings);
        }



        configurePathPlanner();
        // FieldCentricFacingAngleSwerveRequest.HeadingController.setPID()

        // Set up custom logging to add the current path to a field 2d widget
        PathPlannerLogging.setLogActivePathCallback((poses) -> swerveFieldPoseEstimate.getObject("path").setPoses(poses));

        cameraFieldPoseEstimate.setRobotPose(getPose());
        swerveFieldPoseEstimate.setRobotPose(getPose());
        fusedFieldPoseEstimate.setRobotPose(getPose());
        SmartDashboard.putData("cameraFieldPoseEstimate", cameraFieldPoseEstimate);
        SmartDashboard.putData("swerveFieldPoseEstimate", swerveFieldPoseEstimate);
        SmartDashboard.putData("fusedFieldPoseEstimate", fusedFieldPoseEstimate);
        // SmartDashboard.putData("m_pigeon2");

        timer = System.currentTimeMillis();

        
    }

    public void setStartingPose()
    {
        // TODO Set Starting Poes for reference point
        PathPlannerAuto.getStaringPoseFromAutoFile(null);
    }


    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Pose2d getPose()
    {
        return this.m_odometry.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose)
    {
        this.modifiedTareEverything(pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds()
    {
        return m_kinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds)
    {
        chassisSpeedsSwerveRequest.withSpeeds(robotRelativeSpeeds);
        this.setControl(chassisSpeedsSwerveRequest);
    }

    public void xBrakes()
    {
        this.setControl(xBrakeSwerveRequest);
    }

    public void zeroGyo()
    {
        this.seedFieldRelative();
    }

    public void updatePose(Pose2d visionPose2dEstimate, double timestampSeconds)
    {
        this.m_odometry.addVisionMeasurement(visionPose2dEstimate, timestampSeconds);;
        System.out.println("Fused Position with Drivetrain: " + this.m_odometry.getEstimatedPosition());
    }


    // @Override
    // public void periodic() {
    //     // TODO Auto-generated method stub
    //     Subsystem.super.periodic();

    //     // actuatorAngleTarget = Vision.getPitchDegreesToTarget(this.getPose(), Constants.FieldElementsPositions.BlueAllianceSide.Speaker);
    //     // swerveYawTarget = Vision.getYawDegreesToTarget(this.getPose(), Constants.FieldElementsPositions.BlueAllianceSide.Speaker);

    //     // System.out.println("Pitch: " + m_actuatorAngleTarget.getAsDouble());
    //     // System.out.println("Target: " + m_swerveYawTarget.getAsDouble());

    //     //System.out.println("PhotonVision Peroidic");

    //     // swerveFieldPoseEstimate.setRobotPose(this.m_odometry.getEstimatedPosition());
    //     try {
    //         EstimatedRobotPose estimatedPoseResult = CameraSubsystem.getEstimatedRobotPose(getPose());
    //         if (estimatedPoseResult != null)
    //         {
    //             Pose2d robotPose = estimatedPoseResult.estimatedPose.toPose2d();
    //             double timestamp = estimatedPoseResult.timestampSeconds;

    //             System.out.println("Estimated Pose: " + estimatedPoseResult);


    //             // cameraFieldPoseEstimate.setRobotPose(robotPose);

    //             // TODO: More than just this april tag deviation.
    //             Translation2d aprilTagPose = Constants.CameraConstants.aprilTagFieldLayout.getTagPose(estimatedPoseResult.targetsUsed.get(0).getFiducialId()).get().toPose2d().getTranslation();

    //             double distance = this.getPose().getTranslation().getDistance(aprilTagPose);

    //             // System.out.println("April Tag Pose: " + aprilTagPose + "\n -- " + distance);

    //             Matrix<N3, N1>  visionmeasurementStdDevsMatrix = VecBuilder.fill(distance / 2, distance / 2, 100);
    //             this.m_odometry.addVisionMeasurement(robotPose, timestamp, visionmeasurementStdDevsMatrix);
    //             System.out.println("Added Vision Measurement");
    //             // this.m_odometry.addVisionMeasurement(robotPose, timestamp);

    //             // System.out.println("Test: " + Vision.getPitchDegreesToTarget(this.getPose(), Constants.FieldElementsPositions.BlueAllianceSide.Speaker));

    //             if (System.currentTimeMillis() - timer > 500)
    //             {
    //                 timer = System.currentTimeMillis();

    //                 // swerveFieldPoseEstimate.setRobotPose(this.m_odometry.getEstimatedPosition());
    //                 cameraFieldPoseEstimate.setRobotPose(robotPose);
    //                 fusedFieldPoseEstimate.setRobotPose(this.m_odometry.getEstimatedPosition());

    //             }
                
    //         }
    //     } catch (Exception e) {
    //         // TODO: handle exception
    //         System.out.println("No April Tag Detected? . . . " + e);
    //         //e.printStackTrace();
    //     }


    //     CameraSubsystem.updateRobotPose(getPose());

    // }



    /**
     * xVelocity = -joystick.getLeftY()
     * yVelocity = -joystick.getLeftX()
     * rotationalRate = -joystick.getRightX()
     * 
     * @param xVelocity - The percentage of maximum speed in the x-direction such as controller (-1, 1) joystick values.
     * @param yVelocity - The percentage of maximum speed in the y-direction such as controller (-1, 1) joystick values.
     * @param rotationalRate - The percentage of maximum angularVelocity such as controller (-1, 1) joystick values.
     * @return Default Command for SwerveSubsystem
     */
    public Command drive(Supplier<Double> leftY, Supplier<Double> leftX, Supplier<Double> rightX)
    {
        Supplier<SwerveRequest> requestSupplier = () -> FieldCentricSwerveRequest
            .withVelocityX(leftY.get() * Constants.SwerveDriveConstants.MAX_TELE_DRIVE_SPEED) // Drive forward with negative Y (forward)
            .withVelocityY(leftX.get() * Constants.SwerveDriveConstants.MAX_TELE_DRIVE_SPEED) // Drive left with negative X (left)
            .withRotationalRate(rightX.get() * Constants.SwerveDriveConstants.MAX_ANGULAR_RATE); // Drive counterclockwise with negative X (left)

        return this.applyRequest(requestSupplier);
    }



    /**
     * xVelocity = -joystick.getLeftY()
     * yVelocity = -joystick.getLeftX()
     * rotationalRate = -joystick.getRightX()
     * 
     * @param xVelocity - The percentage of maximum speed in the x-direction such as controller (-1, 1) joystick values.
     * @param yVelocity - The percentage of maximum speed in the y-direction such as controller (-1, 1) joystick values.
     * @param targetDirection - The yaw target in degrees to face while driving (no strafe)
     * @return Drive Facing Angle Command for SwerveSubsystem
     */
    public Command driveFacingAngle(Supplier<Double> leftY, Supplier<Double> leftX, Rotation2d targetDirection)
    {
        Supplier<SwerveRequest> requestSupplier = () -> FieldCentricFacingAngleSwerveRequest
            .withVelocityX(leftY.get() * Constants.SwerveDriveConstants.MAX_TELE_DRIVE_SPEED) // Drive forward with negative Y (forward)
            .withVelocityY(leftX.get() * Constants.SwerveDriveConstants.MAX_TELE_DRIVE_SPEED) // Drive left with negative X (left)
            .withTargetDirection(targetDirection);

            return this.applyRequest(requestSupplier);
    }


    /**
     * Zero's this swerve drive's odometry entirely.
     * <p>
     * This will zero the entire odometry, and place the robot at Pose2d
     */
    private void modifiedTareEverything(Pose2d pose)
    {   
        this.tareEverything();
        // TODO: Modify to only reset odometry?
        try {
            m_stateLock.writeLock().lock();

            for (int i = 0; i < ModuleCount; ++i) {
                Modules[i].resetPosition();
                m_modulePositions[i] = Modules[i].getPosition(true);
            }
            m_odometry.resetPosition(Rotation2d.fromDegrees(m_yawGetter.getValue()), m_modulePositions, pose);
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    /**
     * @return
     * An Array of the Swerve Drive Modules States
     */
    public SwerveModuleState[] getModuleStates()
    {
        return this.getState().ModuleStates;
    }

    /**
     * 
     * Configures PathPlanner AutoBuilder for autonomous paths including on-the-fly generation paths.
     */
    private void configurePathPlanner() {

        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    3, // Max module speed, in m/s
                    0.368, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }


    /**
     * 
     * @param targetPose - Position and Orientation of Robot. Ex:--"new Pose2d(10, 5, Rotation2d.fromDegrees(180)"--
     * This example has a target end position of x = 10 meters, y = 5 meters, facing 180 degrees.
     * @param constraints - A PathConstraints object containing the Maximum Translational and Rotational Speeds allowed on path.
     * @return A path from starting point to end point
     */
    public Command generateOnTheFlyPath(Pose2d targetPose, PathConstraints constraints)
    {
        //TODO: Fix This
        // Since we are using a holonomic drivetrain, the rotation component of this pose
        // represents the goal holonomic rotation
        // Pose2d targetPose = new Pose2d(10, 5, Rotation2d.fromDegrees(180));

        // Create the constraints to use while pathfinding
        
        /*
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));
        */

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );

        return pathfindingCommand;
    }





  // All SwerveSubsystem Commands

  public Command CMDxBrakes()
  {
    return runOnce(() -> this.xBrakes());
  }

  public Command CMDzeroGyro()
  {
    return runOnce(() -> this.zeroGyo());
  }

}
