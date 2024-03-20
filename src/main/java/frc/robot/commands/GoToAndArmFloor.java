// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Harvester;

public class GoToAndArmFloor extends ParallelCommandGroup {
  /** Creates a new HarvesterToFloor. */
  Harvester m_harvester;
  DriveSubsystem m_robotDrive;
  Boolean commandOver;
  double armAngleFloor;
  Command driveAndFloor;
  double robotX;
  double robotY;
  double heading;
  
  public void HarvesterToFloor(Harvester harvester, DriveSubsystem drive, double x, double y, double degrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_harvester = harvester;
    m_robotDrive = drive;
    robotY=y;
    robotX=x;
    heading = degrees;
    commandOver = false;
    addRequirements(m_harvester, m_robotDrive);
    addCommands(HarvesterToFloor, GoToCommand);
    driveAndFloor = Commands.parallel(
        Commands.HarvesterToFloor(),
        Commands.GoToCommand())
    ;
    
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  
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
