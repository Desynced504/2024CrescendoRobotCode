// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HarvesterSubsystem extends SubsystemBase {

  private CANSparkMax outsideHarvester = new CANSparkMax(Constants.HarvesterConstants.OuterBar_ID, MotorType.kBrushless);
  private CANSparkMax insideHarvester = new CANSparkMax(Constants.HarvesterConstants.InnerBar_ID, MotorType.kBrushless);

  /** Creates a new HarvesterSubsystem. */
  public HarvesterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void harvest() {
    outsideHarvester.set(0.9);
    insideHarvester.set(-0.9);
  }

  private void reverseHarvest() {
    outsideHarvester.set(-0.4);
    insideHarvester.set(0.4);
  }

  private void stopHarvester() {
    outsideHarvester.set(0);
    insideHarvester.set(0);
  }

  // All HarvesterSubsystem Commands

  public Command CMD_harvest()
  {
    return runOnce(() -> this.harvest());
  }

  public Command CMD_reverseHarvest()
  {
    return runOnce(() -> this.reverseHarvest());
  }

  public Command CMD_stopHarvester()
  {
    return runOnce(() -> this.stopHarvester());
  }
}
