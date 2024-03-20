// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//This command combines intake reverse and shooter cycle in a faster way when it is applicable at the start of the match
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Harvester;
import frc.robot.subsystems.Shooter;

public class StartSpeaker extends Command {
  /** Creates a new StartSpeaker. */
  Shooter m_shooter;
  Harvester m_harvester;
  Boolean commandOver;
  public StartSpeaker(Shooter shooter, Harvester harvester) {
    // Use addRequirements() here to declare subsystem dependencies.
    commandOver = false;
    m_harvester = harvester;
    m_shooter = shooter;
   addRequirements(m_harvester);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setRPM(1500);
    Commands.waitSeconds(3.0);
    //TODO Adjust waittime
    m_harvester.setIntakeSpeed(-1.0);
    Commands.waitSeconds(0.75);
    m_harvester.setIntakeSpeed(0.0);
    m_shooter.setRPM(0);
    commandOver = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
