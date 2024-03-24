// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {

  private CANSparkMax middleBar = new CANSparkMax(Constants.IndexerConstants.Indexer_ID, MotorType.kBrushless);
  private DigitalInput leftIndexerLimitSwitch = new DigitalInput(Constants.IndexerConstants.LeftBarrelLimitSwitch_PWM_ID);
  private DigitalInput rightIndexerLimitSwitch = new DigitalInput(Constants.IndexerConstants.RightBarrelLimitSwitch_PWM_ID);

  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem()
  {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void indexFeed() {
    // System.out.println("Indexer Feeding!");
    middleBar.set(-1.0);
  }

  private void indexReverse() {
    System.out.println("Reversing Indexer!");
    middleBar.set(0.4);
  }

  private void stopIndexer() {
    System.out.println("Stopped Indexer!");
    middleBar.set(0);
  }

  private boolean isNotePresent()
  {
    // System.out.println("Limit Switch State: " + !rightIndexerLimitSwitch.get());
    // System.out.println("Checking if note is Present!");
    return !rightIndexerLimitSwitch.get() || !leftIndexerLimitSwitch.get();
  }

  // All IndexerSubsystem Commands

  public Command CMD_indexFeed()
  {
    return runOnce(() -> this.indexFeed());
  }

  public Command CMD_indexReverse()
  {
    return runOnce(() -> this.indexReverse());
  }

  public Command CMD_stopIndexer()
  {
    return runOnce(() -> this.stopIndexer());
  }

  public Command CMD_isNotePresent()
  {
    return runOnce(() -> this.isNotePresent());
  }

  public Command CMD_waitForNote()
  {
    return run(() -> System.out.println("Waiting.")).until(this::isNotePresent);
  }

  public Command CMD_harvestUntilFull()
  {
    return run(() -> this.indexFeed()).until(this::isNotePresent);
  }
  
  
}
