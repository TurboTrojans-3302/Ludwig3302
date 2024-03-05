// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;


public class TeleopShooter extends Command {
  /** Creates a new TeleopShooter. */
  Shooter m_shooter;
  XboxController m_CopilotController;
  public TeleopShooter(Shooter shooter, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = m_shooter;
    controller = m_CopilotController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.shooterRPM = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  
  @Override
  public void execute() {
    //should work because execute is called every 20ms.
    if (m_CopilotController.getYButton()){
      m_shooter.increaseRPM(true);
    } else if (m_CopilotController.getAButton()){
      m_shooter.decreaseRPM(true);
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.shooterRPM = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
//
