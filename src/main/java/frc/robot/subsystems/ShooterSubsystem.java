// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  //private final TalonFX leftShooterSparky = new TalonFX(Constants.ShooterConstants.Sparky_LeftShooter_ID);
  //private final TalonFX rightShooterZappy = new TalonFX(Constants.ShooterConstants.Zappy_RightShooter_ID);
  private final TalonFX shooterMotor = new TalonFX(Constants.ShooterConstants.Shooter_Motor_ID);

    private final Slot0Configs wheelRPMControllerConfigs = new Slot0Configs()
    .withKP(Constants.ShooterConstants.ShooterVelocity_Kp)
    .withKI(Constants.ShooterConstants.ShooterVelocity_Ki)
    .withKD(Constants.ShooterConstants.ShooterVelocity_Kd)
    .withKS(Constants.ShooterConstants.ShooterVelocity_Ks)
    .withKV(Constants.ShooterConstants.ShooterVelocity_Kv);


  /** Creates a new ShooterMechanismSubsystem. */
  public ShooterSubsystem() {
    shooterMotor.getConfigurator().apply(wheelRPMControllerConfigs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  private void shootNote()
  {
    //TODO: Create Closed Loop
    System.out.println("Spinning Shooter Wheels!");
    shooterMotor.set(-1);
    System.out.println(shooterMotor.getVelocity());
  }

  private void shootTrap()
  {
    //TODO: Create Closed Loop
    System.out.println("Spinning Shooter Wheels!");
    shooterMotor.set(-0.5);
    System.out.println(shooterMotor.getVelocity());
  }

  private boolean areWheelsAboveVelocityThreshold()
  {
    double shotSpeed, shotTargetSpeed;
    shotSpeed = Math.abs(shooterMotor.getRotorVelocity().getValueAsDouble());

    shotTargetSpeed = Math.abs(Constants.ShooterConstants.DefaultShotPercent * 100);

    System.out.println("Shot Target Speed: " + shotTargetSpeed + "   -   Shooter Current Velocity:" + shotSpeed);

    // Return true when within 80% of target shot speed.
    return (shotSpeed > shotTargetSpeed * 0.8);
  }



  private void stopShooting()
  {
    System.out.println("Stopping Shooter Wheels!");
    shooterMotor.set(0);
  }

  // All ShooterSubsystem Commands

  public Command CMD_shootNote()
  {
    return runOnce(() -> this.shootNote());
  }

  public Command CMD_trapShot()
  {
    return runOnce(() -> this.shootTrap());
  }

  public Command CMD_stopShooting()
  {
    return runOnce(() -> this.stopShooting());
  }

  public Command CMD_shootNoteAtTargetVelocity()
  {
    return runOnce(() -> this.shootNote()).repeatedly().until(this::areWheelsAboveVelocityThreshold);
  }



}
