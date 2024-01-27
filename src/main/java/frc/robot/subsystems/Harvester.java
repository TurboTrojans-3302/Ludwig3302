// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Harvester extends SubsystemBase {

  public static final double ANGLE_AT_FLOOR = -5.0;
  public static final double ANGLE_AT_AMP = 45.0;
  public static final double ANGLE_AT_SPEAKER = 100;
  public static final double ANGLE_AT_DRIVE = 90;

  /** Creates a new Harvester. */
  public Harvester() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean hasNote() {
    //TODO
    return false;
  }

  public void setHarvestSpeed(double speed) {
    //TODO
    
  }


  public double getHarvesterSeed() {
    //TODO
    return 0.0;
  }

  public double getHarvesterAngle() {
    //TODO
    return 0.0;
  }

  public void setHarvesterAngle(double angle) {
    //TODO

  }

  public void setArmAtFloor(double angle) {
    //TODO
  }

  public boolean getArmAtFloor() {
    return ANGLE_AT_FLOOR == getHarvesterAngle();
  }

  public void setArmAtAmp(double angle) {
    //TODO
  }

  public boolean getArmAtAmp() {
    return ANGLE_AT_AMP == getHarvesterAngle();
  }

  public void setArmAtSpeaker(double angle) {
    //TODO
  }

  public boolean getArmAtSpeaker() {
    return ANGLE_AT_SPEAKER == getHarvesterAngle();
  }
 }
