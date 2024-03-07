// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.counter.Tachometer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private VictorSPX m_leftMotor, m_rightMotor;
  private PIDController mLeftPidController, mRightPidController;
  private Tachometer mLeftTachometer, mRightTachometer;
  private Double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  private ShuffleboardTab mShuffleboardTab;
  private Double mSetpoint, mLeftVelocity, mRightVelocity;

  /** Creates a new Shooter. */
  public Shooter() {
    m_leftMotor = new VictorSPX(Constants.ShooterConstants.kShooterLeftCanId);
    m_rightMotor = new VictorSPX(Constants.ShooterConstants.kShooterRightCanId);

    mLeftTachometer  = new Tachometer(new DigitalInput(Constants.ShooterConstants.kLeftTachDIO));
    mRightTachometer = new Tachometer(new DigitalInput(Constants.ShooterConstants.kRightTachDIO));
    mLeftTachometer.setEdgesPerRevolution(2048);
    mRightTachometer.setEdgesPerRevolution(2048);

    // PID coefficients
    kP = 6e-5; 
    kI = 0.0;
    kD = 0.0; 
    kIz = 0.0; 
    kFF = 0.000015; 
    maxRPM = 5700.0;

    mLeftPidController = new PIDController(kP, kI, kD);
    mRightPidController =  new PIDController(kP, kI, kD);
    mLeftPidController.setTolerance(Constants.ShooterConstants.RPM_TOLERANCE);
    mRightPidController.setTolerance(Constants.ShooterConstants.RPM_TOLERANCE);

    mSetpoint = 0.0;


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
             .withProperties(Map.of("min", (Constants.ShooterConstants.RPM_TOLERANCE * -5.0),
                                    "max", (Constants.ShooterConstants.RPM_TOLERANCE * 5.0),
                                    "Show value", false));
    rpmLayout.add("Err R", mRightVelocity)
             .withWidget(BuiltInWidgets.kDial)
             .withProperties(Map.of("min", (Constants.ShooterConstants.RPM_TOLERANCE * -5.0),
                                    "max", (Constants.ShooterConstants.RPM_TOLERANCE * 5.0),
                                    "Show value", false));

  }

  @Override
  public void periodic() {

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP))   { mLeftPidController.setP(p); kP = p; }
    if((i != kI))   { mLeftPidController.setI(i); kI = i; }
    if((d != kD))   { mLeftPidController.setD(d); kD = d; }
    if((iz != kIz)) { mLeftPidController.setIZone(iz); kIz = iz; }

    mLeftVelocity  = mLeftTachometer.getRevolutionsPerMinute();
    mRightVelocity = mRightTachometer.getRevolutionsPerMinute();

    double rightOutput = mRightPidController.calculate(mRightVelocity, mSetpoint);
    double leftOutput = mLeftPidController.calculate(mLeftVelocity, mSetpoint);

    m_leftMotor.set(VictorSPXControlMode.PercentOutput, MathUtil.clamp(leftOutput, -1.0, 1.0));
    m_rightMotor.set(VictorSPXControlMode.PercentOutput, MathUtil.clamp(rightOutput, -1.0, 1.0));

    SmartDashboard.putNumber("Setpoint", mSetpoint);
    SmartDashboard.putNumber("Err L", errL());
    SmartDashboard.putNumber("Err R", errR());
  
    SmartDashboard.putBoolean("ShooterReady", speedIsReady());
  }
  
  public void setRPM(double speed) {
    mSetpoint = MathUtil.clamp(speed, 0.0, maxRPM);
  }

  public double getRPM(){
    return mSetpoint;
  }
  
  private Double errL() { return mLeftVelocity - mSetpoint; };
  private Double errR() { return mRightVelocity - mSetpoint; };

  public boolean speedIsReady(){
    return mRightPidController.atSetpoint() && mLeftPidController.atSetpoint();
  }

}


