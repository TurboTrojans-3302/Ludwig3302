// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Harvester;


public class TeleopHarvester extends Command {
  private XboxController m_controller;
  private Harvester m_harvester;
  private double mSetpoint;

  /** Creates a new TeleopHarvester. */
  public TeleopHarvester( Harvester harvester, XboxController controller) {
    m_controller = controller;
    m_harvester = harvester;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_harvester);
    mSetpoint = m_harvester.getArmAngle();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mSetpoint = m_harvester.getArmAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double harvesterSpeed;
    double armSpeed;

     if (m_controller.getRightBumper()) {
      harvesterSpeed = Constants.harvesterConstants.inSpeed;
     }
     else if (m_controller.getLeftBumper()) {
      harvesterSpeed = Constants.harvesterConstants.outSpeed;
     }
     else {
      harvesterSpeed = 0.0;
     }
    m_harvester.setIntakeSpeed(harvesterSpeed);

    armSpeed = (m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis());
    mSetpoint += armSpeed * 2.0;
    m_harvester.setArmAngle(mSetpoint);

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
