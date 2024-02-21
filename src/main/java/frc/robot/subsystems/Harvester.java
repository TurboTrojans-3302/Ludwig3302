// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Harvester extends SubsystemBase {

  private VictorSPX m_intakeSpx;
  private CANSparkMax m_armSpx;
  private AbsoluteEncoder m_ArmEncoder;

  private DigitalInput mBackLimitSwitch;
  private PIDController mPid;

  /** Creates a new Harvester. */
  public Harvester() {
    m_armSpx = new CANSparkMax(Constants.harvesterConstants.kArmLiftCanId, MotorType.kBrushless);
    m_armSpx.setIdleMode(IdleMode.kBrake);
    m_ArmEncoder = m_armSpx.getAbsoluteEncoder(Type.kDutyCycle);
    m_ArmEncoder.setPositionConversionFactor(360.0);
    
    m_intakeSpx = new VictorSPX(Constants.harvesterConstants.kIntakeCanId);
    m_intakeSpx.setNeutralMode(NeutralMode.Brake);
    mBackLimitSwitch = new DigitalInput(Constants.harvesterConstants.kBackLimitSwitchInputID);

    mPid = new PIDController(0.01, 0.0, 0.0);
    mPid.setTolerance(Constants.harvesterConstants.ANGLE_TOLERANCE);

    mPid.setSetpoint(getArmAngle());
  }

  
  @Override
  public void periodic() {
    double speed = mPid.calculate(getArmAngle());
    //TODO implement some rate limiting here
    setArmSpeed(speed);
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
    return m_ArmEncoder.getPosition();
  }

  public double getArmSetpoint() {
    return mPid.getSetpoint();
  }

  public void setArmAngle(double angle) {
    mPid.setSetpoint(MathUtil.clamp(angle, 
                                    Constants.harvesterConstants.ANGLE_MIN,
                                    Constants.harvesterConstants.ANGLE_MAX));
  }

  private void setArmSpeed(double speed){
    m_armSpx.set(speed);
  }

  public boolean isArmAtAngle(double angle) {
    return MathUtil.isNear(angle, getArmAngle(), Constants.harvesterConstants.ANGLE_TOLERANCE);
  }
 }
