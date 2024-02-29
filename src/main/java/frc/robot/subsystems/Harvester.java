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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Harvester extends SubsystemBase {

  private VictorSPX m_intakeSpx;
  private CANSparkMax m_armSpx;
  private DutyCycleEncoder m_ArmEncoder;

  private DigitalInput mBackLimitSwitch;

  private final ShuffleboardTab m_shuffleboardTab;
  private final GenericEntry m_armAngleEntry;
  private final GenericEntry m_hasNoteEntry;

  /** Creates a new Harvester. */
  public Harvester() {
    m_armSpx = new CANSparkMax(Constants.harvesterConstants.kArmLiftCanId, MotorType.kBrushless);
    m_armSpx.setIdleMode(IdleMode.kBrake);
    m_ArmEncoder = new DutyCycleEncoder(Constants.harvesterConstants.kArmEncoderDInput);
    m_ArmEncoder.setDistancePerRotation(360.0);
    m_ArmEncoder.setPositionOffset(Constants.harvesterConstants.armEncoderOffset);
    
    m_intakeSpx = new VictorSPX(Constants.harvesterConstants.kIntakeCanId);
    m_intakeSpx.setNeutralMode(NeutralMode.Brake);
    mBackLimitSwitch = new DigitalInput(Constants.harvesterConstants.kBackLimitSwitchInputID);

    m_shuffleboardTab = Shuffleboard.getTab("Harvester");
    m_armAngleEntry = m_shuffleboardTab.add("Arm Angle", 0.0)
                                .withWidget(BuiltInWidgets.kGyro)
                                .withProperties(Map.of("StartingAngle", 90.0))
                                .getEntry();
    m_hasNoteEntry = m_shuffleboardTab.add("Have Note", false)
                                .withWidget(BuiltInWidgets.kBooleanBox)
                                .getEntry();
  }

  
  @Override
  public void periodic() {
    m_armAngleEntry.setDouble(getArmAngle());
    m_hasNoteEntry.setBoolean(hasNote());
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
    return m_ArmEncoder.getDistance();
  }

  public void setArmAngle(double angle) {
    //TODO

  }

  public void setArmSpeed(double speed){
    m_armSpx.set(speed);
  }

  public boolean isArmAtAngle(double angle) {
    return MathUtil.isNear(angle, getArmAngle(), Constants.harvesterConstants.ANGLE_TOLERANCE);
  }
 }
