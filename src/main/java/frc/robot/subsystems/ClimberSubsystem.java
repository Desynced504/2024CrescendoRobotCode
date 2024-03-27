// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  // Motor Directions, Positive is extend, Negative is retract
  private TalonFX climber = new TalonFX(Constants.ClimberConstants.Climber_ID);

  private final SoftwareLimitSwitchConfigs climberLimits = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(75)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(0);

  private final MotorOutputConfigs climberMotorOutputConfigs = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake);

  // TODO: Find Constants for Climber including Static Friction or Feedforward, Potentially add jerk to make it smoother.
  // TODO: MUST TEST IN TUNER !!!

  private final MotionMagicConfigs climberMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicAcceleration(Constants.ClimberConstants.MagicAcceleration)
            .withMotionMagicCruiseVelocity(Constants.ClimberConstants.MagicCruiseVelocity);

  private final Slot0Configs climberSlot0Configs = new Slot0Configs()
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKP(Constants.ClimberConstants.Climber_Kp)
            .withKI(Constants.ClimberConstants.Climber_Ki)
            .withKD(Constants.ClimberConstants.Climber_Kd)
            .withKS(Constants.ClimberConstants.Climber_Ks)
            .withKV(Constants.ClimberConstants.Climber_Kv)
            .withKA(Constants.ClimberConstants.Climber_Ka);

  /** Creates a new ActuatorSubsystem. */
  public ClimberSubsystem() {
    // this.applyClimberConstants();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Assuming fully retracted on startup;
   * 0 rotations = fully retracted
   * 75 rotations = fully extended
   * @param targetRotations - amount of rotations falcon spins. (Not accounting for gear ratios in configs)
   */
  public void setClimberToTargetPosition(double targetRotations)
  {
    MotionMagicDutyCycle motionRequest = new MotionMagicDutyCycle(targetRotations);
    climber.setControl(motionRequest);

    System.out.println("Climber Current Position: " + climber.getPosition());
  }

  private void applyClimberConstants()
  {
    // climber.getConfigurator().apply(climberMotorOutputConfigs);
    // climber.getConfigurator().apply(climberLimits);
    // climber.getConfigurator().apply(climberSlot0Configs);
    // climber.getConfigurator().apply(climberMotionMagicConfigs);
  }

  /**
   * Sets Climber Target Position (MotionMagicControl) to the input target position.
   * @param target - Requested Position of Climber (0 = retracted, 75 = extended)
   * @return
   */
  public Command CMD_setClimberTo(double target)
  {

    return runOnce(() -> this.setClimberToTargetPosition(target));
  }

  public Command extend()
  {
    return run(() -> climber.set(0.4));
  }

  public Command retract()
  {
    return run(() -> climber.set(-0.4));
  }

  public Command stopClimber()
  {
    return run(() -> climber.set(0));
  }
}
