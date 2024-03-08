// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.counter.Tachometer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private VictorSPX m_leftMotor, m_rightMotor;
  private PIDController mLeftPidController, mRightPidController;
  private Tachometer mLeftTachometer, mRightTachometer;
  private Double kP, kI, kD, kIz, kFF, maxRPM;
  private Double mSetpoint, mLeftVelocity, mRightVelocity;
  private AnalogInput mUltrasonicInput;

  
  private ShuffleboardTab mShuffleboardTab;
  private GenericEntry mSetpointEntry, mLeftErrEntry, mRightErrEntry,
                       mSpeedReadyEntry, mUltrasonicInputEntry,
                       mLeftVelEntry, mRightVelEntry, mPIDEntry,
                       mLeftOutputEntry, mRightOuputEntry;
  
  /** Creates a new Shooter. */
  public Shooter() {
    m_leftMotor = new VictorSPX(Constants.ShooterConstants.kShooterLeftCanId);
    m_rightMotor = new VictorSPX(Constants.ShooterConstants.kShooterRightCanId);

    mLeftTachometer  = new Tachometer(new DigitalInput(Constants.ShooterConstants.kLeftTachDIO));
    mRightTachometer = new Tachometer(new DigitalInput(Constants.ShooterConstants.kRightTachDIO));
    mLeftTachometer.setEdgesPerRevolution(2048);
    mRightTachometer.setEdgesPerRevolution(2048);

    mUltrasonicInput = new AnalogInput(Constants.ShooterConstants.kShooterUltrasonicAIO);
    mUltrasonicInput.setAverageBits(4);

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
    mLeftVelocity = 0.0;
    mRightVelocity = 0.0;


    // display PID coefficients on SmartDashboard
    mShuffleboardTab = Shuffleboard.getTab("Shooter");
    mSpeedReadyEntry = mShuffleboardTab.add("ShooterReady", speedIsReady())
                                       .withWidget(BuiltInWidgets.kBooleanBox)
                                       .getEntry();
    mSetpointEntry = mShuffleboardTab.add("Setpoint", mSetpoint).getEntry();
    mLeftErrEntry = mShuffleboardTab.add("Err L", errL())
             .withWidget(BuiltInWidgets.kDial)
             .withProperties(Map.of("min", (Constants.ShooterConstants.RPM_TOLERANCE * -5.0),
                                    "max", (Constants.ShooterConstants.RPM_TOLERANCE * 5.0),
                                    "Show value", true))
             .getEntry();
    mRightErrEntry = mShuffleboardTab.add("Err R", errR())
             .withWidget(BuiltInWidgets.kDial)
             .withProperties(Map.of("min", (Constants.ShooterConstants.RPM_TOLERANCE * -5.0),
                                    "max", (Constants.ShooterConstants.RPM_TOLERANCE * 5.0),
                                    "Show value", true))
             .getEntry();
    mUltrasonicInputEntry = mShuffleboardTab.add("Range", mUltrasonicInput.getAverageValue())
                                            .withWidget(BuiltInWidgets.kTextView)
                                            .getEntry();
    mLeftVelEntry = mShuffleboardTab.add("Left Velocity", mLeftTachometer.getRevolutionsPerMinute())
                                    .withWidget(BuiltInWidgets.kField)
                                    .getEntry();
    mRightVelEntry = mShuffleboardTab.add("Right Velocity", mRightTachometer.getRevolutionsPerMinute())
                                    .withWidget(BuiltInWidgets.kField)
                                     .getEntry();
    mLeftOutputEntry = mShuffleboardTab.add("Left Motor Output", m_leftMotor.getMotorOutputPercent())
                                    .withWidget(BuiltInWidgets.kField)
                                     .getEntry();
    mRightOuputEntry = mShuffleboardTab.add("Right Motor Output", m_rightMotor.getMotorOutputPercent())
                                    .withWidget(BuiltInWidgets.kField)
                                     .getEntry();
    mShuffleboardTab.add("Left PID", mLeftPidController)
                        .withWidget(BuiltInWidgets.kPIDController);
  }

  @Override
  public void periodic() {

    mLeftVelocity  = mLeftTachometer.getRevolutionsPerMinute();
    mRightVelocity = mRightTachometer.getRevolutionsPerMinute();

    double rightOutput = mRightPidController.calculate(mRightVelocity, mSetpoint);
    double leftOutput = mLeftPidController.calculate(mLeftVelocity, mSetpoint);

    m_leftMotor.set(VictorSPXControlMode.PercentOutput, MathUtil.clamp(leftOutput, -1.0, 1.0));
    m_rightMotor.set(VictorSPXControlMode.PercentOutput, MathUtil.clamp(rightOutput, -1.0, 1.0));

    mSetpointEntry.setDouble(mSetpoint);
    mLeftErrEntry.setDouble(errL());
    mRightErrEntry.setDouble(errR());
    mSpeedReadyEntry.setBoolean(speedIsReady());
    mUltrasonicInputEntry.setDouble(mUltrasonicInput.getAccumulatorValue());
    mLeftVelEntry.setDouble(mLeftVelocity);
    mRightVelEntry.setDouble(mRightVelocity);
    mLeftOutputEntry.setDouble(m_leftMotor.getMotorOutputPercent());
    mRightOuputEntry.setDouble(m_rightMotor.getMotorOutputPercent());
    
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

  public double sensorDistance(){
    return mUltrasonicInput.getAverageValue();
  }
}


