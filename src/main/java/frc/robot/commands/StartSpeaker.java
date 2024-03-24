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
   addRequirements(m_shooter, m_harvester);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setRPM(3302.0);
    System.out.println("setpoint is: " + m_shooter.getRPMsetpoint());
    System.out.println("velocity L" + m_shooter.mLeftVelocity);
    Commands.waitSeconds(10.0);
    System.out.println("waiting for rpm");
    System.out.println("setpoint is: " + m_shooter.getRPMsetpoint());
    System.out.println("velocity L" + m_shooter.mLeftVelocity);
    Commands.waitUntil(()->{ return m_shooter.speedIsReady();});
    System.out.println("ready!");
    m_harvester.setIntakeSpeed(-1.0);
    Commands.waitSeconds(1.0);
    m_shooter.setRPM(0.0);
    m_harvester.setIntakeSpeed(0.0);
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
