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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DoNothing;
import frc.robot.commands.SetArmAngleCommand;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.TeleopHarvester;
import frc.robot.commands.TeleopShooter;
import frc.robot.commands.GoToCommand;
import frc.robot.commands.Auto1Blue;
import frc.robot.commands.Auto1Red;
import frc.robot.commands.Auto2;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.DriveDashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Harvester;
import frc.robot.subsystems.Shooter;
 

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final DriveDashboard mDriveDashboard = new DriveDashboard(m_robotDrive);  
  public final Harvester m_harvester = new Harvester();
  public final Shooter m_shooter = new Shooter();
  private final Climbers m_climbers = new Climbers();
  Command Auto1B = new Auto1Blue(m_robotDrive, m_harvester, m_shooter);
  Command Auto1R = new Auto1Red(m_robotDrive, m_shooter, m_harvester);
  Command Speaker2 = new Auto2(m_robotDrive, m_shooter, m_harvester);



  private final ShuffleboardTab m_shuffleboardTab;
  private final SendableChooser<Command> m_autonomousChooser;
  private final SendableChooser<Pose2d> m_startPosChooser;

  private final REVBlinkinLED m_BlinkinLED;

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
    m_robotDrive.setDefaultCommand(new TeleopDrive(m_robotDrive, m_driverController, mDriveDashboard));

    m_harvester.setDefaultCommand(new TeleopHarvester(m_harvester, m_driverController)); 
    
    m_shooter.setDefaultCommand(new TeleopShooter(m_shooter, m_copilotController));

    m_climbers.setDefaultCommand(new ClimbCommand(m_climbers, m_copilotController));

    m_shuffleboardTab = Shuffleboard.getTab("Game");
    
    m_autonomousChooser = new SendableChooser<Command>();
    m_autonomousChooser.setDefaultOption("Do Nothing", new DoNothing());
    m_autonomousChooser.addOption("CrossTheLine", GoToCommand.relative(m_robotDrive, 1.0, 0.0, 90.0));
    m_autonomousChooser.addOption("Speaker and Amp - Center - Blue", Auto1B);
    m_autonomousChooser.addOption("Speaker and Amp - Center - Red", Auto1R);
    m_autonomousChooser.addOption("Speaker twice - Center", Speaker2);
    m_shuffleboardTab.add("Auton Command", m_autonomousChooser);

    m_startPosChooser = new SendableChooser<Pose2d>();
    m_startPosChooser.setDefaultOption("Left", Constants.FieldConstants.StartPositionLeft);
    m_startPosChooser.addOption("Center", Constants.FieldConstants.StartPositionCenter);
    m_startPosChooser.addOption("Right", Constants.FieldConstants.StartPositionRight);
    m_shuffleboardTab.add("Start Position", m_startPosChooser);

    m_shuffleboardTab.add("Floor", new SetArmAngleCommand(m_harvester, Constants.harvesterConstants.ANGLE_AT_FLOOR));
    m_shuffleboardTab.add("Amp", new SetArmAngleCommand(m_harvester, Constants.harvesterConstants.ANGLE_AT_AMP));
    m_shuffleboardTab.add("30", new SetArmAngleCommand(m_harvester, 30.0));
    m_shuffleboardTab.add("Drive", new SetArmAngleCommand(m_harvester, Constants.harvesterConstants.ANGLE_AT_DRIVE));
    m_shuffleboardTab.add("Speaker", new SetArmAngleCommand(m_harvester, Constants.harvesterConstants.ANGLE_AT_SPEAKER));
    m_BlinkinLED = new REVBlinkinLED(Constants.BLINKIN_LED_PWM_CHANNEL);
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
    new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    new JoystickButton(m_driverController, XboxController.Button.kRightStick.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .onTrue(new SetArmAngleCommand(m_harvester, Constants.harvesterConstants.ANGLE_AT_SPEAKER));
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .onTrue(new SetArmAngleCommand(m_harvester, Constants.harvesterConstants.ANGLE_AT_DRIVE));
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .onTrue(new SetArmAngleCommand(m_harvester, Constants.harvesterConstants.ANGLE_AT_AMP));
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .onTrue(new SetArmAngleCommand(m_harvester, Constants.harvesterConstants.ANGLE_AT_FLOOR));
  }

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

  public void setLED(double value) {
    m_BlinkinLED.set(value);
  }
}
