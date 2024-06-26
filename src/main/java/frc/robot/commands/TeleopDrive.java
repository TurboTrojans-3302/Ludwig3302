// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveDashboard;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopDrive extends Command {
  private DriveSubsystem m_robotDrive;
  private XboxController m_driverController;
  private DriveDashboard m_DriveDashboard;

  /** Creates a new TeleopDrive. */
  public TeleopDrive(DriveSubsystem robotDrive, XboxController driverController, DriveDashboard dash) {
    m_driverController = driverController;
    m_robotDrive = robotDrive;
    m_DriveDashboard = dash;
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_robotDrive.setAll(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightTriggerAxis = m_driverController.getRightTriggerAxis();
    if (rightTriggerAxis < 0.5){
    m_robotDrive.drive(
                    stick2speed(-m_driverController.getLeftY()),
                    stick2speed(-m_driverController.getLeftX()),
                    stick2speed(-m_driverController.getRightX()),
                    true,
                    false);
    } 
    else if (rightTriggerAxis > 0.5) {
      m_robotDrive.drive(
                    stick2speed(-0.5 * m_driverController.getLeftY()),
                    stick2speed(-0.5 * m_driverController.getLeftX()),
                    stick2speed(-0.5 * m_driverController.getRightX()),
                    true,
                    false);}

    
    int pov = m_driverController.getPOV();

    switch (pov) {
      case 0:
        m_robotDrive.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        break;
    
      case 90:
        m_robotDrive.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(270)));
        break;
    
      case 180:
        m_robotDrive.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(180)));
        break;
    
      case 270:
        m_robotDrive.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(90)));
        break;
    
      default:
        break;
    }
                
  }

  // applies deadband and scaling to raw stick value
  private double stick2speed(double stickValue) {
    return Math.signum(stickValue) * Math.pow(MathUtil.applyDeadband(stickValue, OIConstants.kDriveDeadband), 2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
