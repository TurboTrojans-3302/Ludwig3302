// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DoNothing;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.TeleopHarvester;
import frc.robot.commands.TranslateCommand;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.DriveDashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Harvester;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  @SuppressWarnings("unused")
  private final DriveDashboard mDriveDashboard = new DriveDashboard(m_robotDrive);  
  private final Harvester m_harvester = new Harvester();
  private final Climbers m_climbers = new Climbers();


  private final ShuffleboardTab m_shuffleboardTab;
  private final SendableChooser<Command> m_autonomousChooser;
  private final SendableChooser<Pose2d> m_startPosChooser;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_copilotController = new XboxController(OIConstants.kCopilotControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(new TeleopDrive(m_robotDrive, m_driverController));

    m_harvester.setDefaultCommand(new TeleopHarvester(m_harvester, m_driverController)); 

    m_climbers.setDefaultCommand(new ClimbCommand(m_climbers, m_copilotController));
    
    m_shuffleboardTab = Shuffleboard.getTab("Game");
    
    m_autonomousChooser = new SendableChooser<Command>();
    m_autonomousChooser.setDefaultOption("Do Nothing", new DoNothing());
    m_autonomousChooser.addOption("CrossTheLine", new TranslateCommand(m_robotDrive, 1.0, 0.0));
    m_shuffleboardTab.add("Auton Command", m_autonomousChooser);

    m_startPosChooser = new SendableChooser<Pose2d>();
    m_startPosChooser.setDefaultOption("Left", Constants.FieldConstants.StartPositionLeft);
    m_startPosChooser.addOption("Center", Constants.FieldConstants.StartPositionCenter);
    m_startPosChooser.addOption("Right", Constants.FieldConstants.StartPositionRight);
    m_shuffleboardTab.add("Start Position", m_startPosChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setAll(0.0, 0.0),
            m_robotDrive));
  } //TODO add brake indicator to dashboard

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonomousChooser.getSelected();
  }

  public Pose2d getStartPosition() {
    return m_startPosChooser.getSelected();
  }

  public void setStartPosition() {
    m_robotDrive.resetOdometry(getStartPosition());
  }
}
