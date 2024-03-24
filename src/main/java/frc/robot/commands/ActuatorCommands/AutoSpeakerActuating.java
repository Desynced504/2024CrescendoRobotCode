// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.ActuatorCommands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ActuatorSubsystem;
// import frc.robot.subsystems.CameraSubsystem;

// public class AutoSpeakerActuating extends Command {

//   private final ActuatorSubsystem m_ActuatorSubsystem;
//   private PIDController m_actuatorPIDController = new PIDController(2, 0.5, 0);
//   private double m_actuatorAngle;
//   private double m_targetAngle;
//   private double m_targetMotorSpeed;

//   /** Creates a new ActuatorAngleTargetting. */
//   public AutoSpeakerActuating(ActuatorSubsystem actuatorSubsystem, Pose3d targetPose3d) {
//     double targetAngle = CameraSubsystem.getActuatorTargetAngle(targetPose3d);
//     // Restrict Target Angle from 30 to 70
//     targetAngle = targetAngle < 30 ? 30:targetAngle;
//     targetAngle = targetAngle > 70 ? 70:targetAngle;

//     m_targetAngle = targetAngle;

//     m_ActuatorSubsystem = actuatorSubsystem;
//     addRequirements(m_ActuatorSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize()
//   {
//     System.out.println("Before Running: " + m_targetAngle);



//     m_actuatorPIDController.setTolerance(1);
//     // m_actuatorPIDController.reset();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute()
//   {
//     m_actuatorAngle = m_ActuatorSubsystem.gyroRollSupplier();
//     System.out.println("Raw Gyro Data: " + m_actuatorAngle);

//     double output = m_actuatorPIDController.calculate(m_actuatorAngle, m_targetAngle);

//     output = Math.min(output, 20);
//     output = Math.max(output, -20);
//     System.out.println("Output: " + output);
//     m_targetMotorSpeed = (output / 20);

//     m_ActuatorSubsystem.setActuatorSpeed(m_targetMotorSpeed);

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted)
//   {
//     m_ActuatorSubsystem.setActuatorSpeed(0);
//     System.out.println("TargetAngle: " + m_targetAngle);
//     System.out.println("Finished Calculation: " + (Math.abs(Math.abs(m_actuatorAngle) - Math.abs(m_targetAngle))));
//     System.out.println("Finished Actuating!");
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return (Math.abs(Math.abs(m_actuatorAngle) - Math.abs(m_targetAngle)) < 1);
//   }
// }
