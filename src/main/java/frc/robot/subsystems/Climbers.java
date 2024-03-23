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

public class Climbers extends SubsystemBase {

  private VictorSPX m_climberRight;
  private VictorSPX m_climberLeft;
  private DigitalInput mRightBottomLimit;
  private DigitalInput mLeftBottomLimit;
  private DigitalInput mRightTopLimit;
  private DigitalInput mLeftTopLimit;


  /** Creates a new Climbers. */
  public Climbers() {
    m_climberRight = new VictorSPX(Constants.ClimberConstants.rightClimberCanID);
    m_climberLeft = new VictorSPX(Constants.ClimberConstants.leftClimberCanID);
    m_climberRight.setNeutralMode(NeutralMode.Brake);
    m_climberLeft.setNeutralMode(NeutralMode.Brake);
    mRightBottomLimit = new DigitalInput(Constants.ClimberConstants.rightBottomLimitDIO);
    mLeftBottomLimit = new DigitalInput(Constants.ClimberConstants.leftBottomLimitDIO);
    mRightTopLimit = new DigitalInput(Constants.ClimberConstants.rightTopLimitDIO);
    mLeftTopLimit = new DigitalInput(Constants.ClimberConstants.leftTopLimitDIO);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climberRightMove(double speed) {
    //if(rightFullyContracted()){ speed = Math.max(speed, 0.0); }
  m_climberRight.set(VictorSPXControlMode.PercentOutput, speed);

  }

  public void climberLeftMove(double speed) {
    //if(leftFullyContracted()){ speed = Math.max(speed, 0.0); }
    m_climberLeft.set(VictorSPXControlMode.PercentOutput, speed);
  }

  public boolean leftFullyContracted(){ return mLeftBottomLimit.get(); }
  public boolean rightFullyContracted(){ return mRightBottomLimit.get(); }
}
