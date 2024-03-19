// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Harvester;
import frc.robot.subsystems.Shooter;



public class IntakeAndToShooter extends Command {


 
 private boolean commandOver = false;


 /** Creates a new Climbers. */
 Harvester m_harvester;
 Shooter m_shooter;
 Double armAngle;
 Double angleSpeaker;


 public IntakeAndToShooter(Harvester harvester, Shooter shooter) {
   // Use addRequirements() here to declare subsystem dependencies.
   m_harvester = harvester;
   m_shooter = shooter;
   armAngle = m_harvester.getArmSetpoint();
   
   angleSpeaker = Constants.harvesterConstants.ANGLE_AT_SPEAKER;
   
   addRequirements(m_harvester, m_shooter);


 }


 // Called when the command is initially scheduled.
 @Override
 public void initialize() {

   m_harvester.setIntakeSpeed(1.0);
   Commands.waitSeconds(0.5);
   m_harvester.setArmAngle(angleSpeaker);
   Commands.waitSeconds(0.5);
   m_harvester.setIntakeSpeed(0.0);

   


   
 }



 // Called every time the scheduler runs while the command is scheduled.
 @Override
 public void execute() {
    if (m_harvester.isArmAtAngle() && m_harvester.hasNote()){
        commandOver = true;
    }

   
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
