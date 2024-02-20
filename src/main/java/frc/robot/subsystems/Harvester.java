// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Harvester extends SubsystemBase {

  private VictorSPX m_intakeSpx;
  private VictorSPX m_armSpx;

  private DigitalInput mBackLimitSwitch;

  /** Creates a new Harvester. */
  public Harvester() {
    m_armSpx = new VictorSPX(Constants.harvesterConstants.kIntakeArmLift);
    m_intakeSpx = new VictorSPX(Constants.harvesterConstants.kIntakeCanId);

    m_armSpx.setNeutralMode(NeutralMode.Brake);
    m_intakeSpx.setNeutralMode(NeutralMode.Brake);

    mBackLimitSwitch = new DigitalInput(Constants.harvesterConstants.kBackLimitSwitchInputID);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean hasNote() {
    return !mBackLimitSwitch.get(); 
  }

  public void setIntakeSpeed(double speed) {
    double limitedSpeed = MathUtil.clamp(speed, (hasNote() ? 0.0 : -1.0), 1.0);
    m_intakeSpx.set(VictorSPXControlMode.PercentOutput, limitedSpeed);
  }


  public double getIntakeSpeed() {
    return m_intakeSpx.getMotorOutputPercent();
  }

  public double getArmAngle() {
    //TODO
    return 0.0;
  }

  public void setArmAngle(double angle) {
    //TODO

  }

  public void setArmSpeed(double speed){
    m_armSpx.set(VictorSPXControlMode.PercentOutput, speed);
  }

  public boolean isArmAtAngle(double angle) {
    return MathUtil.isNear(angle, getArmAngle(), Constants.harvesterConstants.ANGLE_TOLERANCE);
  }
 }
