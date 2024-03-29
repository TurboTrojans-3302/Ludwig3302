// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.commands.GoToCommand;
import frc.robot.commands.SetArmAngleCommand;
import frc.robot.commands.SetIntakeCommand;
import frc.robot.commands.SpinUpShooter;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.TeleopHarvester;
import frc.robot.commands.TeleopShooter;
import frc.robot.commands.GoToCommand;
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

    m_harvester.setDefaultCommand(new TeleopHarvester(m_harvester, m_copilotController)); 
    
    m_shooter.setDefaultCommand(new TeleopShooter(m_shooter, m_copilotController));

    m_climbers.setDefaultCommand(new ClimbCommand(m_climbers, m_copilotController));

    m_shuffleboardTab = Shuffleboard.getTab("Game");
    
    m_autonomousChooser = new SendableChooser<Command>();
    m_autonomousChooser.setDefaultOption("forward then left", GoToCommand.absolute(m_robotDrive, 1.5, 0.0, 0.0)
                                                                  .andThen(GoToCommand.absolute(m_robotDrive, 1.5, 3.0, 90)));
    
    m_autonomousChooser.addOption("Just Shoot", new SpinUpShooter(m_shooter, 3302)
                                            .andThen(new SetIntakeCommand(m_harvester,
                                                                Constants.harvesterConstants.outSpeed,
                                                           0.75)));
    m_shuffleboardTab.add("Auton Command", m_autonomousChooser);

    m_startPosChooser = new SendableChooser<Pose2d>();
    m_startPosChooser.setDefaultOption("ZeroZero", Constants.FieldConstants.ZeroZero);
    m_startPosChooser.addOption("Left +30", new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(30.0)));
    m_startPosChooser.addOption("Right -30", new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(-30.0)));
    m_shuffleboardTab.add("Start Position", m_startPosChooser);

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

    new JoystickButton(m_copilotController, XboxController.Button.kX.value)
        .onTrue(new SetArmAngleCommand(m_harvester, Constants.harvesterConstants.ANGLE_AT_SPEAKER));
    new JoystickButton(m_copilotController, XboxController.Button.kY.value)
        .onTrue(new SetArmAngleCommand(m_harvester, Constants.harvesterConstants.ANGLE_AT_DRIVE));
    new JoystickButton(m_copilotController, XboxController.Button.kB.value)
        .onTrue(new SetArmAngleCommand(m_harvester, Constants.harvesterConstants.ANGLE_AT_AMP));
    new JoystickButton(m_copilotController, XboxController.Button.kA.value)
        .onTrue(new SetArmAngleCommand(m_harvester, Constants.harvesterConstants.ANGLE_AT_FLOOR));

    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(() -> m_harvester.setIntakeSpeed(Constants.harvesterConstants.inSpeed),
                                 m_harvester));
    
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new RunCommand(() -> {
                                          if(m_harvester.getArmAngle() > 90){
                                              m_harvester.setIntakeSpeed(Constants.harvesterConstants.outSpeed);
                                          }else{
                                              m_harvester.setIntakeSpeed(Constants.harvesterConstants.outSpeedSlow);
                                          }
                                        }, m_harvester));                             
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
