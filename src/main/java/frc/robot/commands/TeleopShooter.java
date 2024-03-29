// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.MotControllerJNI;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;


public class TeleopShooter extends Command {
  /** Creates a new TeleopShooter. */
  Shooter m_shooter;
  XboxController m_CopilotController;
  public TeleopShooter(Shooter shooter, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_CopilotController = controller;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setRPM(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  
  @Override
  public void execute() {
    //should work because execute is called every 20ms.
    int pov = m_CopilotController.getPOV();
    switch (pov) {
      case 0:
        m_shooter.setRPM(m_shooter.getRPMsetpoint()+50);
        break;
    
      case 90:
        m_shooter.setRPM(0);
        break;
    
      case 180:
        m_shooter.setRPM(m_shooter.getRPMsetpoint()-50);
        break;
    
      case 270:
        m_shooter.setRPM(3302);
        break;
    
      default:
        break;
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setRPM(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
//
