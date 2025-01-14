// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.Optional;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    DataLogManager.start();
    CameraServer.startAutomaticCapture();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.setLED(REVBlinkinLED.Pattern.SOLID_VIOLET);
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    setLED(LEDmode.Auton);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    System.out.println("Auton: " + m_autonomousCommand.getName());
    m_robotContainer.setStartPosition();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    setLED(LEDmode.Teleop);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(m_robotContainer.m_harvester.hasNote()){
      if(m_robotContainer.m_shooter.speedIsReady()){
        setLED(LEDmode.Ready2Shoot);
      } else {
        setLED(LEDmode.HaveNote);
      }
    } else {
      setLED(LEDmode.Teleop);
    }    

    if(30.0 > DriverStation.getMatchTime() && DriverStation.getMatchTime() > 29.0){
      m_robotContainer.m_driverController.setRumble(RumbleType.kBothRumble, 1.0);
      m_robotContainer.m_copilotController.setRumble(RumbleType.kBothRumble, 1.0);
    }else{
      m_robotContainer.m_driverController.setRumble(RumbleType.kBothRumble, 0.0);
      m_robotContainer.m_copilotController.setRumble(RumbleType.kBothRumble, 0.0);
    }

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  private enum LEDmode {
    Auton,
    Teleop,
    HaveNote,
    Ready2Shoot
  }

  Map<Alliance, Map<LEDmode, Double>> ledPatternMap = Map.of(
    Alliance.Red, Map.of(
      LEDmode.Auton, REVBlinkinLED.Pattern.COLOR1_LARSON_SCANNER,
      LEDmode.Teleop, REVBlinkinLED.Pattern.SOLID_RED,
      LEDmode.HaveNote, REVBlinkinLED.Pattern.COLOR1_HEARTBEAT_MEDIUM,
      LEDmode.Ready2Shoot, REVBlinkinLED.Pattern.COLOR1_HEARTBEAT_FAST
    ),
    Alliance.Blue, Map.of(
      LEDmode.Auton, REVBlinkinLED.Pattern.COLOR2_LARSON_SCANNER,
      LEDmode.Teleop, REVBlinkinLED.Pattern.SOLID_BLUE,
      LEDmode.HaveNote, REVBlinkinLED.Pattern.COLOR2_HEARTBEAT_MEDIUM,
      LEDmode.Ready2Shoot, REVBlinkinLED.Pattern.COLOR2_HEARTBEAT_FAST
    )
  );

  private void setLED(LEDmode mode){
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if(alliance.isPresent()){
      m_robotContainer.setLED(ledPatternMap.get(alliance.get()).get(mode));
    } else {
      m_robotContainer.setLED(REVBlinkinLED.Pattern.COLOR1_AND_2_TWINKLES_COLOR1_AND_2);
    }
  }
}
