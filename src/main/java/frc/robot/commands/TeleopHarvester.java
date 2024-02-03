// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Harvester;


public class TeleopHarvester extends Command {
  private XboxController m_controller;
  private Harvester m_harvester;

  /** Creates a new TeleopHarvester. */
  public TeleopHarvester( Harvester harvester, XboxController controller) {
    m_controller = controller;
    m_harvester = harvester;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double harvesterSpeed;
     if (m_controller.getRightBumper()) {
      harvesterSpeed = Constants.harvesterConstants.inSpeed;
     }
     else if (m_controller.getLeftBumper()) {
      harvesterSpeed = Constants.harvesterConstants.outSpeed;
     }
     else {
      harvesterSpeed = 0.0;
     }
    m_harvester.setHarvestSpeed(harvesterSpeed);

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
