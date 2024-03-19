// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Harvester;

public class HarvesterToFloor extends Command {
  /** Creates a new HarvesterToFloor. */
  Harvester m_harvester;
  Boolean commandOver;
  Double armAngleFloor;
  public HarvesterToFloor(Harvester harvester) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_harvester = harvester;
    commandOver = false;
    armAngleFloor = Constants.harvesterConstants.ANGLE_AT_FLOOR;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_harvester.setArmAngle(armAngleFloor);
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_harvester.isArmAtAngle()){
      commandOver = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return commandOver;
  }
}
