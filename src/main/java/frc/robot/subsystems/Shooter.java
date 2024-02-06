// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public Shooter() {
    CANSparkMax m_leftMotor = new CANSparkMax(Constants.ShooterConstants.kShooterLeftCanId, MotorType.kBrushless);
    CANSparkMax m_rightMotor = new CANSparkMax(Constants.ShooterConstants.kShooterRightCanId, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public double getSpeed() {    
    //TODO
    return 0.0;
  }

  public void setSpeed(double speed) {
    //TODO 
  }

  public boolean speedIsReady(){
    return false; //TODO finish me!!!
  }

}


