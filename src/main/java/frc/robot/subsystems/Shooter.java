// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private CANSparkMax m_leftMotor, m_rightMotor;
  private PIDController mLeftPidController, mRightPidController;
  private SimpleMotorFeedforward mSimpleMotorFeedforward;
  private RelativeEncoder mLeftEncoder, mRightEncoder;
  private Double kP, kI, kD, maxRPM, kDwellTime;
  private Double kV;
  public Double mSetpoint;
  public Double mLeftVelocity;
  private Double mRightVelocity;
  private AnalogInput mUltrasonicInput;
  private Timer timer = new Timer();

  private ShuffleboardTab mShuffleboardTab;
  private GenericEntry mSetpointEntry, mLeftErrEntry, mRightErrEntry,
                       mSpeedReadyEntry, mUltrasonicInputEntry,
                       mLeftVelEntry, mRightVelEntry,
                       mLeftOutputEntry, mRightOuputEntry;
  
  /** Creates a new Shooter. */
  public Shooter() {
    m_leftMotor = new CANSparkMax(Constants.ShooterConstants.kShooterLeftCanId, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(Constants.ShooterConstants.kShooterRightCanId, MotorType.kBrushless);
    m_leftMotor.setInverted(true);
    m_rightMotor.setInverted(false);

    mLeftEncoder  = m_leftMotor.getEncoder();
    mRightEncoder = m_leftMotor.getEncoder();
    mLeftEncoder.setVelocityConversionFactor(1.0); 
    mRightEncoder.setVelocityConversionFactor(1.0);

    mUltrasonicInput = new AnalogInput(Constants.ShooterConstants.kShooterUltrasonicAIO);
    mUltrasonicInput.setAverageBits(4);

    timer.restart();

    // PID coefficients
    kP = 0.0003; 
    kI = 0.000001;
    kD = 0.00001; 
    maxRPM = 5700.0;
    kDwellTime = 0.020;

    //FF constants
    kV = 0.6/3302.0;
    


    mLeftPidController = new PIDController(kP, kI, kD);
    mRightPidController =  new PIDController(kP, kI, kD);
    mLeftPidController.setTolerance(Constants.ShooterConstants.RPM_TOLERANCE);
    mRightPidController.setTolerance(Constants.ShooterConstants.RPM_TOLERANCE);

    mSimpleMotorFeedforward = new SimpleMotorFeedforward(0.0, kV, 0.0);

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
    mLeftVelEntry = mShuffleboardTab.add("Left Velocity", mLeftEncoder.getVelocity())
                                    .withWidget(BuiltInWidgets.kTextView)
                                    .getEntry();
    mRightVelEntry = mShuffleboardTab.add("Right Velocity", mRightEncoder.getVelocity())
                                    .withWidget(BuiltInWidgets.kTextView)
                                     .getEntry();
    mLeftOutputEntry = mShuffleboardTab.add("Left Motor Output", m_leftMotor.get())
                                    .withWidget(BuiltInWidgets.kTextView)
                                     .getEntry();
    mRightOuputEntry = mShuffleboardTab.add("Right Motor Output", m_rightMotor.get())
                                    .withWidget(BuiltInWidgets.kTextView)
                                     .getEntry();
    // mShuffleboardTab.add("Left PID", mLeftPidController)
    //                     .withWidget(BuiltInWidgets.kPIDController);
  }

  @Override
  public void periodic() {

    // if(m_leftMotor.getFaults() != 0){
    //   for (FaultID f : FaultID.values()) { 
    //      if(m_leftMotor.getFault(f)) { System.out.println("Left Shooter Motor Fault: " + f);}
    //   }
    // }

    // if(m_rightMotor.getFaults() != 0){
    //   for (FaultID f : FaultID.values()) { 
    //      if(m_rightMotor.getFault(f)) { System.out.println("Right Shooter Motor Fault: " + f);}
    //   }
    // }

    mLeftVelocity  = mLeftEncoder.getVelocity();
    mRightVelocity = mRightEncoder.getVelocity();

    double ff = mSimpleMotorFeedforward.calculate(mSetpoint);

    double rightOutput = ff + mRightPidController.calculate(mRightVelocity, mSetpoint);
    double leftOutput = ff + mLeftPidController.calculate(mLeftVelocity, mSetpoint);

    if(mSetpoint <= 0){
      rightOutput = 0.0;
      leftOutput = 0.0;
    }

    m_leftMotor.set(MathUtil.clamp(leftOutput, -1.0, 1.0));
    m_rightMotor.set(MathUtil.clamp(rightOutput, -1.0, 1.0));

    if(!atSetpoint()){ timer.restart(); }
    
    mSetpointEntry.setDouble(mSetpoint);
    mLeftErrEntry.setDouble(errL());
    mRightErrEntry.setDouble(errR());
    mSpeedReadyEntry.setBoolean(speedIsReady());
    mUltrasonicInputEntry.setDouble(mUltrasonicInput.getAccumulatorValue());
    mLeftVelEntry.setInteger(mLeftVelocity.intValue());
    mRightVelEntry.setInteger(mRightVelocity.intValue());
    mLeftOutputEntry.setDouble(m_leftMotor.get());
    mRightOuputEntry.setDouble(m_rightMotor.get());
    
  }
  
  public void setRPM(double speed) {
    mSetpoint = MathUtil.clamp(speed, 0.0, maxRPM);
  }

  public double getRPMsetpoint(){
    return mSetpoint;
  }
  
  public Double errL() { return mLeftVelocity - mSetpoint; };
  public Double errR() { return mRightVelocity - mSetpoint; };

  public boolean atSetpoint(){
    return mRightPidController.atSetpoint() && mLeftPidController.atSetpoint();
  }

  public boolean speedIsReady(){
    return atSetpoint() && timer.hasElapsed(kDwellTime);
  }

  public double sensorDistance(){
    return mUltrasonicInput.getAverageValue();
  }
}


