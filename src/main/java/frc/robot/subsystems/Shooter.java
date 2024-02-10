// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private final Double RPM_TOLERANCE = 20.0;

  private CANSparkMax m_leftMotor, m_rightMotor;
  private SparkPIDController mLeftPidController, mRightPidController;
  private RelativeEncoder mLeftRelativeEncoder, mRightRelativeEncoder;
  private Double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  private ShuffleboardTab mShuffleboardTab;
  private Double mSetpoint, mLeftVelocity, mRightVelocity;

  /** Creates a new Shooter. */
  public Shooter() {
    m_leftMotor = new CANSparkMax(Constants.ShooterConstants.kShooterLeftCanId, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(Constants.ShooterConstants.kShooterRightCanId, MotorType.kBrushless);

    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();

    /**
     * In order to use PID functionality for a controller, a SparkPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    mLeftPidController = m_leftMotor.getPIDController();
    mRightPidController = m_leftMotor.getPIDController();

    // Encoder object created to display position values
    mLeftRelativeEncoder = m_leftMotor.getEncoder();
    mRightRelativeEncoder = m_rightMotor.getEncoder();

    // PID coefficients
    kP = 6e-5; 
    kI = 0.0;
    kD = 0.0; 
    kIz = 0.0; 
    kFF = 0.000015; 
    kMaxOutput = 1.0; 
    kMinOutput = -1.0;
    maxRPM = 5700.0;

    mSetpoint = 0.0;

    // set PID coefficients
    mLeftPidController.setP(kP);
    mLeftPidController.setI(kI);
    mLeftPidController.setD(kD);
    mLeftPidController.setIZone(kIz);
    mLeftPidController.setFF(kFF);
    mLeftPidController.setOutputRange(kMinOutput, kMaxOutput);
    mRightPidController.setP(kP);
    mRightPidController.setI(kI);
    mRightPidController.setD(kD);
    mRightPidController.setIZone(kIz);
    mRightPidController.setFF(kFF);
    mRightPidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    mShuffleboardTab = Shuffleboard.getTab("Shooter");
    mShuffleboardTab.add("P Gain", kP);
    mShuffleboardTab.add("I Gain", kI);
    mShuffleboardTab.add("D Gain", kD);
    mShuffleboardTab.add("I Zone", kIz);
    mShuffleboardTab.add("Feed Forward", kFF);
    mShuffleboardTab.add("Max Output", kMaxOutput);
    mShuffleboardTab.add("Min Output", kMinOutput);

    mShuffleboardTab.add("ShooterReady", speedIsReady()).withWidget(BuiltInWidgets.kBooleanBox);

    ShuffleboardLayout rpmLayout = mShuffleboardTab.getLayout("RPM");
    rpmLayout.add("Setpoint", mSetpoint);
    rpmLayout.add("Err L", mLeftVelocity)
             .withWidget(BuiltInWidgets.kDial)
             .withProperties(Map.of("min", (RPM_TOLERANCE * -5.0), "max", (RPM_TOLERANCE * 5.0), "Show value", false));
    rpmLayout.add("Err R", mRightVelocity)
             .withWidget(BuiltInWidgets.kDial)
             .withProperties(Map.of("min", (RPM_TOLERANCE * -5.0), "max", (RPM_TOLERANCE * 5.0), "Show value", false));

  }

  @Override
  public void periodic() {

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP))   { mLeftPidController.setP(p); kP = p; }
    if((i != kI))   { mLeftPidController.setI(i); kI = i; }
    if((d != kD))   { mLeftPidController.setD(d); kD = d; }
    if((iz != kIz)) { mLeftPidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { mLeftPidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      mLeftPidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    mLeftVelocity  =  mLeftRelativeEncoder.getVelocity();
    mRightVelocity = mRightRelativeEncoder.getVelocity();

    mLeftPidController.setReference(mSetpoint, CANSparkMax.ControlType.kVelocity);
    mRightPidController.setReference(mSetpoint, CANSparkMax.ControlType.kVelocity);
    
    SmartDashboard.putNumber("Setpoint", mSetpoint);
    SmartDashboard.putNumber("Err L", errL());
    SmartDashboard.putNumber("Err R", errR());
  
    SmartDashboard.putBoolean("ShooterReady", speedIsReady());
  }
  
  public void setSpeed(double speed) {
    mSetpoint = speed;
  }

  private Double errL() { return mLeftVelocity - mSetpoint; };
  private Double errR() { return mRightVelocity - mSetpoint; };

  public boolean speedIsReady(){
    double errL = Math.abs(errL());
    double errR = Math.abs(errR());
    return (errL < RPM_TOLERANCE) && (errR < RPM_TOLERANCE);
  }

}


