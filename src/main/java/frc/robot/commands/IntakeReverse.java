// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Harvester;
import frc.robot.subsystems.Shooter;


public class IntakeReverse extends Command {


 
 private boolean commandOver = false;


 /** Creates a new Climbers. */
 Harvester m_harvester;


 public IntakeReverse(Harvester harvester) {
   // Use addRequirements() here to declare subsystem dependencies.
   m_harvester = harvester;
   addRequirements(m_harvester);


 }


 // Called when the command is initially scheduled.
 @Override
 public void initialize() {
   m_harvester.setIntakeSpeed(-0.75);
   Commands.waitSeconds(1.0);
   m_harvester.setIntakeSpeed(0.0);
   Commands.waitSeconds(0.1);
   commandOver = true;
 }


 // Called every time the scheduler runs while the command is scheduled.
 @Override
 public void execute() {


 }


 // Called once the command ends or is interrupted.
 @Override
 public void end(boolean interrupted) {
  
 }


 // Returns true when the command should end.
 @Override
 public boolean isFinished() {
   return commandOver;
 }
}
