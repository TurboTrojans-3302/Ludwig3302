// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Harvester;

public class HarvesterToFloor extends ParallelCommand {
  /** Creates a new HarvesterToFloor. */
  Harvester m_harvester;
  Drivetrain m_robotDrive;
  Boolean commandOver;
  Double armAngleFloor;
  Command driveAndFloor;
  public HarvesterToFloor(Harvester harvester, Drivetrain drive, double x, double y, double degrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_harvester = harvester;
    m_robotDrive = drive;
    commandOver = false;
    
    driveAndFloor = Commands.parallel(
        Commands.HarvesterToFloor(m_harvester),
        Commands.GoToCommand(m_robotDrive, new Pose2D(x, y, Rotation2D.fromDegrees(degrees)));


    )
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveAndFloor;
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
