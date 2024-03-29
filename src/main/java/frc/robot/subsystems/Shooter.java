// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.function.Consumer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private CANSparkMax m_leftMotor, m_rightMotor;
  private PIDController mLeftPidController, mRightPidController;
  private RelativeEncoder mLeftEncoder, mRightEncoder;
  private Double kP, kI, kD, maxRPM, kDwellTime;
  public Double mSetpoint;
  public Double mLeftVelocity;
  private Double mRightVelocity;
  private AnalogInput mUltrasonicInput;
  private Timer timer = new Timer();
  private boolean mPidEnabled = true;
  private SysIdRoutine mIdRoutine;
  
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
    kP = 0.0005; 
    kI = 0.0001;
    kD = 0.00005; 
    maxRPM = 5700.0;
    kDwellTime = 0.020;


    mLeftPidController = new PIDController(kP, kI, kD);
    mRightPidController =  new PIDController(kP, kI, kD);
    mLeftPidController.setTolerance(Constants.ShooterConstants.RPM_TOLERANCE);
    mRightPidController.setTolerance(Constants.ShooterConstants.RPM_TOLERANCE);

    mSetpoint = 0.0;
    mLeftVelocity = 0.0;
    mRightVelocity = 0.0;

    mIdRoutine = new SysIdRoutine(new Config(),
                                  new Mechanism( this::sysIdVoltageDrive, this::sysIdLog, this));


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

    if(mPidEnabled){
      double rightOutput = mRightPidController.calculate(mRightVelocity, mSetpoint);
      double leftOutput = mLeftPidController.calculate(mLeftVelocity, mSetpoint);

      if(mSetpoint <= 0){
        rightOutput = 0.0;
        leftOutput = 0.0;
      }

      m_leftMotor.set(MathUtil.clamp(leftOutput, -1.0, 1.0));
      m_rightMotor.set(MathUtil.clamp(rightOutput, -1.0, 1.0));
    }

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

  public void sysIdLog(SysIdRoutineLog log){
     // Record a frame for the shooter motor.
     log.motor("shooter-left")
        .voltage( Volts.of(m_leftMotor.getAppliedOutput() * m_leftMotor.getBusVoltage()))
        .angularVelocity(RPM.of(mLeftEncoder.getVelocity()));
     log.motor("shooter-right")
        .voltage( Volts.of(m_rightMotor.getAppliedOutput() * m_rightMotor.getBusVoltage()))
        .angularVelocity(RPM.of(mRightEncoder.getVelocity()));
  }

  public void sysIdVoltageDrive(Measure<Voltage> volts){
    mPidEnabled = false;
    m_leftMotor.setVoltage(volts.in(Volts));
    m_rightMotor.setVoltage(volts.in(Volts));
  }                                                  

  public void setPidEnabled(boolean pidEnabled) {
    this.mPidEnabled = pidEnabled;
  }

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return mIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return mIdRoutine.dynamic(direction);
  }
}


