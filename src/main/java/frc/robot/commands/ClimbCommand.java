// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Climbers;

public class ClimbCommand extends Command {
  /** Creates a new Climbers. */
  Climbers m_climbers;
  XboxController m_controller;

  public ClimbCommand(Climbers climber, XboxController controller ) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climbers = climber;
    m_controller = controller;
    addRequirements(m_climbers);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double lspeed = 0.0;
    double rspeed = 0.0;
    double lBumper = 0.0;
    double rBumper = 0.0;

  if (m_controller.getLeftBumperPressed()){
    lBumper = 0.5;
  }
  else {
    lBumper = 0.0;
  }
  if (m_controller.getRightBumperPressed()){
    rBumper = 0.5;
  }
  else {
    rBumper = 0.0;
  }
    rspeed = m_controller.getRightTriggerAxis() - rBumper;
    m_climbers.climberRightMove(rspeed);

    lspeed = m_controller.getLeftTriggerAxis() - lBumper;
    m_climbers.climberLeftMove(lspeed);

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
