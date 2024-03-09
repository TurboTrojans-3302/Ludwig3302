// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Harvester extends SubsystemBase {

  private VictorSPX m_intakeSpx;
  private CANSparkMax m_armSpx;
  private DutyCycleEncoder m_ArmEncoder;

  private DigitalInput mBackLimitSwitch;
  private PIDController mPid;

  private final ShuffleboardTab m_shuffleboardTab;
  private final GenericEntry m_armAngleEntry;
  private final GenericEntry m_hasNoteEntry;
  private final GenericEntry mArmMotorEntry;

  private double mVelocities[] = {0.0, 0.0, 0.0, 0.0};
  private double mPreviousAngle;
  private int velIdx = 0;

  /** Creates a new Harvester. */
  public Harvester() {
    m_armSpx = new CANSparkMax(Constants.harvesterConstants.kArmLiftCanId, MotorType.kBrushless);
    m_armSpx.setIdleMode(IdleMode.kBrake);
    m_ArmEncoder = new DutyCycleEncoder(Constants.harvesterConstants.kArmEncoderDInput);
    m_ArmEncoder.setDistancePerRotation(-360.0);
    m_ArmEncoder.setPositionOffset(Constants.harvesterConstants.armEncoderOffset);
    
    m_intakeSpx = new VictorSPX(Constants.harvesterConstants.kIntakeCanId);
    m_intakeSpx.setNeutralMode(NeutralMode.Brake);
    mBackLimitSwitch = new DigitalInput(Constants.harvesterConstants.kBackLimitSwitchInputID);

    mPid = new PIDController(0.001, 0.0, 0.0);
    mPid.setTolerance(Constants.harvesterConstants.ANGLE_TOLERANCE);

    mPid.setSetpoint(getArmAngle());
    mPreviousAngle = getArmAngle();

    m_shuffleboardTab = Shuffleboard.getTab("Harvester");
    m_armAngleEntry = m_shuffleboardTab.add("Arm Angle", 0.0)
                                .withWidget(BuiltInWidgets.kGyro)
                                .withProperties(Map.of("StartingAngle", 90.0,
                                                       "Counter clockwise", true))
                                .getEntry();
    m_hasNoteEntry = m_shuffleboardTab.add("Have Note", false)
                                .withWidget(BuiltInWidgets.kBooleanBox)
                                .getEntry();
    mArmMotorEntry = m_shuffleboardTab.add("Arm Motor", 0.0)
                                .withWidget(BuiltInWidgets.kNumberBar)
                                .withProperties(Map.of("max", 1.0, "min", -1.0))
                                .withPosition(0, 3).withSize(2, 1)
                                .getEntry();
  }

  
  @Override
  public void periodic() {
    double motorcommand = mPid.calculate(getArmAngle());
    setArmMotorPctOutput(MathUtil.clamp(motorcommand, -1.0, 1.0));

    double currentAngle = getArmAngle();
    mVelocities[velIdx] = (currentAngle - mPreviousAngle) / Robot.kDefaultPeriod;
    velIdx = (velIdx + 1) % mVelocities.length;
    mPreviousAngle = currentAngle;

    m_armAngleEntry.setDouble(currentAngle);
    m_hasNoteEntry.setBoolean(hasNote());
    mArmMotorEntry.setDouble(getArmMotorOutput());
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

  public double getArmMotorOutput(){ return m_armSpx.get(); }

  public double getArmAngle() {
    double armangle = m_ArmEncoder.getDistance();
    if (armangle < 180 ){
      return armangle;
    } else {
      return armangle-360;
    }
  }

  public double getArmVelocity(){
    double sum = 0;
    for (double value : mVelocities) {
        sum += value;
    }
    return sum / mVelocities.length;  
  }

  public double getArmSetpoint() {
    return mPid.getSetpoint();
  }

  public void setArmAngle(double angle) {
    mPid.setSetpoint(MathUtil.clamp(angle, 
                                    Constants.harvesterConstants.ANGLE_AT_FLOOR,
                                    Constants.harvesterConstants.ANGLE_AT_SPEAKER));
  }

  private void setArmMotorPctOutput(double speed){
    m_armSpx.set(speed);
  }

  public boolean isArmAtAngle(double angle) {
    return MathUtil.isNear(angle, getArmAngle(), Constants.harvesterConstants.ANGLE_TOLERANCE);
  }
 }
