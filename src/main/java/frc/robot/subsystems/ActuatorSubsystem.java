// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ActuatorSubsystem extends SubsystemBase {
  // Motor Directions, Positive is extend, Negative is retract
  private final TalonFX actuator = new TalonFX(Constants.ActuatorConstants.Actuator_ID);
  private final Pigeon2 gyro = new Pigeon2(Constants.ActuatorConstants.ActuatorGyro_Pigeon_ID);
  // private final LinearFilter gyroRollFilter;

  // TODO: Add Actuator to Interrupt All

  /** Creates a new ActuatorSubsystem. */
  public ActuatorSubsystem() {

    // gyroRollFilter = LinearFilter.movingAverage(10);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double gyroRollSupplier()
  {
    return Math.abs(gyro.getRoll().getValueAsDouble());
  }


  // public DoubleSupplier filteredGyroRollSupplier()
  // {
  //   double averagedAngle = gyroRollFilter.calculate(gyroRollSupplier().getAsDouble());
  //   System.out.println("Averaged Angle: " + averagedAngle);
  //   double roundedAngle = (Math.round(averagedAngle*2) / 2); // Round to Closest 0.5
  //   System.out.println("Rounded Angle: " + roundedAngle);
  //   return () -> averagedAngle;
  // }

  /**
   * 
   * @param percentSpeed - Desired Speed before restrictions
   * @param maxSpeed - Positive Value, Max Speed of Actuator as a percent
   * @param deadband - Positive value from 0 to restrict speeds until value is exceeded. Ex: If deadband is 0.1, a speed of 0.05 will become 0.
   * @return
   */
  public double restrictSpeeds(double percentSpeed, double maxSpeed, double deadband)
  {
    double restrictedSpeed = (Math.abs(percentSpeed) < deadband)? 0 :
                    (percentSpeed > maxSpeed)? maxSpeed :
                    (percentSpeed < -maxSpeed)? -maxSpeed : percentSpeed;
    // TODO: Return long statement
    // System.out.println("RestrictedSpeeds: " + restrictedSpeed);
    return restrictedSpeed;
  }

  /**
   * @param percentSpeed - Value of -1, 1 representing actuator percent output
   */
  public void setActuatorSpeed(double percentSpeed)
  {
    double restrictedSpeed = restrictSpeeds(percentSpeed, 0.5, 0.02);
    System.out.println("Restricted Speed = " + restrictedSpeed);
    actuator.set(restrictedSpeed);
  }

  public Command zeroEncoder()
  {
    actuator.getConfigurator().clearStickyFault_RemoteSensorReset();
    actuator.getPosition();
    return null;
  }

  private boolean isSwitchPressed()
  {
    return actuator.getReverseLimit().getValueAsDouble() == 0;
  }

  public Command testLimitSwitch()
  {
    return run(() -> System.out.println(isSwitchPressed()));
  }


  // public Command spinMotorTo(Supplier<Double> target)
  // {
  //   return run(() -> actuator.setControl(new PositionDutyCycle(((target.get()+1) * 130) + 10, 0, false, 0, 0, true, false, false)));
  // }

  public Command spinMotorTo(double target)
  {

    return runOnce(() -> actuator.setControl(new PositionDutyCycle(target, 0, false, 0, 0, true, false, false)));
  }

  // public Command ActuateToTargetAngle(DoubleSupplier targetAngle)
  // {
  //   return new PIDCommand(
  //       // The controller that the command will use
  //       new PIDController(1, 0.1, 0),
  //       // This should return the measurement
  //       filteredGyroRollSupplier(),
  //       // This should return the setpoint (can also be a constant)
  //       targetAngle,
  //       // This uses the output
  //       output -> {
  //           System.out.println("CurrentAngle: " + filteredGyroRollSupplier().getAsDouble());
  //           // Process Raw Output
  //           output = Math.min(output, 20);
  //           output = Math.max(output, -20);
  //           System.out.println("Output: " + output);
  //           double targetMotorSpeed = (output / 80);

  //           targetMotorSpeed = Math.min(targetMotorSpeed, 0.5);
  //           targetMotorSpeed = Math.max(targetMotorSpeed, -0.5);

  //           if (Math.abs(targetMotorSpeed) < 0.05)
  //               targetMotorSpeed = 0;

  //           System.out.println("Calculated Output: " + targetMotorSpeed);
  //           // actuator.set(targetMotorSpeed);
  //       },
  //       this);
  // }

}
