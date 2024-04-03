// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.PhotonVisionSwerveUtil;
import frc.robot.Constants;
import frc.robot.FieldElements;
import frc.robot.commands.ActuatorCommands.ActuatorAngleTargetting;
import frc.robot.subsystems.ActuatorSubsystem;

public class AutomaticTargetAligner extends Command {

  private final SwerveSubsystem m_SwerveSubsystem;
  private final ActuatorSubsystem m_ActuatorSubsystem;
  private final PhotonVisionSwerveUtil m_PhotonVisionSwerveUtil;
  private final ActuatorAngleTargetting m_ActuatorAngleTargettingCMD;
  private boolean m_alignActuator, m_alignSwerve;
  private FieldElements m_target;
  private Supplier<Double> m_leftY, m_leftX;
  private DoubleSupplier actuatorTargetPitch;
  private Rotation2d aprilTagSwerveHeading;
  private double lastTimeStamp;



  /** Creates a new AutomaticTargetAligner. */
  public AutomaticTargetAligner(SwerveSubsystem swerveSubsystem, ActuatorSubsystem actuatorSubsystem, PhotonVisionSwerveUtil photonVisionSwerveUtil, FieldElements target, boolean alignActuator, boolean alignSwerve, Supplier<Double> leftY, Supplier<Double> leftX) {
    m_SwerveSubsystem = swerveSubsystem;
    m_ActuatorSubsystem = actuatorSubsystem;
    m_PhotonVisionSwerveUtil = photonVisionSwerveUtil;
    m_target = target;
    m_alignSwerve = alignSwerve;
    m_alignActuator = alignActuator;
    m_leftY = leftY;
    m_leftX = leftX;
    actuatorTargetPitch = () -> Constants.ActuatorConstants.automTapeShotAngle;

    m_ActuatorAngleTargettingCMD = new ActuatorAngleTargetting(m_ActuatorSubsystem, actuatorTargetPitch);

    addRequirements(m_SwerveSubsystem, m_ActuatorSubsystem);

    // if (alignActuator && alignSwerve)
    // {
    //   addRequirements(m_SwerveSubsystem, m_ActuatorSubsystem);
    // }
    // else if (alignActuator)
    // {
    //   addRequirements(m_ActuatorSubsystem);
    // }
    // else
    // {
    //   addRequirements(m_SwerveSubsystem);
    // }
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_ActuatorAngleTargettingCMD.initialize();
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {

    if (m_alignActuator)
    {
      // Update Target Pitch Supplier
      actuatorTargetPitch = () -> PhotonVisionSwerveUtil.getActuatorPitchDegreesToTarget(m_SwerveSubsystem.getPose(), m_target.getPose());

      // System.out.println("Target Actuator: " + actuatorTargetPitch.getAsDouble());
      m_ActuatorAngleTargettingCMD.execute();
      m_ActuatorAngleTargettingCMD.updateTargetAngle(actuatorTargetPitch.getAsDouble());

      // m_ActuatorSubsystem.ActuateToTargetAngle(() -> targetPitch);
      // TODO: Command to Set Actuator Angle, perhaps encoder ticks will offer more precision than % output on PID loop.
      // m_ActuatorSubsystem.set . . . 
    }

    if (m_alignSwerve)
    {
      
      Rotation2d poseSwerveDirection = PhotonVisionSwerveUtil.getRotationToTarget(m_SwerveSubsystem.getPose(), m_target.getPose());
      // System.out.println("Target Rotation: " + swerveDirection);
      // System.out.println("Robot Direction: " + m_SwerveSubsystem.getPose().getRotation());


      // try {
      //   Optional<Double> yawOptional = m_PhotonVisionSwerveUtil.getShooterCamera().getYawToTargetAprilTag(m_target.getAprilTagTarget());

      //   if (yawOptional.isPresent())
      //   {
      //     System.out.println(yawOptional.get());
      //     System.out.println("Updating April Tag Yaw!!!");
      //     Rotation2d yawOffset = new Rotation2d(Math.toRadians(yawOptional.get()));
      //     aprilTagSwerveHeading = m_SwerveSubsystem.getPose().getRotation().minus(yawOffset);
      //     lastTimeStamp = System.currentTimeMillis();
      //   }
      // } catch (Exception e) {
      //   // TODO: handle exception
      // }

      // if (System.currentTimeMillis() - lastTimeStamp < 3000)
      // {
      //   System.out.println("Using AprilTag Yaw!!!");
      //   m_SwerveSubsystem.driveFacingAngle(m_leftY, m_leftX, aprilTagSwerveHeading);
      // }
      // else
      // {
        // System.out.println("Using Pose Estimation Yaw!!!");
        m_SwerveSubsystem.driveFacingAngle(m_leftY, m_leftX, poseSwerveDirection);
      // }
      
      // m_SwerveSubsystem.run(() -> m_SwerveSubsystem.driveFacingAngle(m_leftY, m_leftX, swerveDirection));
      // m_SwerveSubsystem.setDefaultCommand
      // (
      //   m_SwerveSubsystem.driveFacingAngle(
      //     m_leftY
      //     , m_leftX
      //     , swerveDirection)
      // );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_ActuatorSubsystem.setActuatorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
