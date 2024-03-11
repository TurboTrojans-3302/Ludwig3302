// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climbers;



public class Climb2 extends Command {
  /** Creates a new Climb2. */

  Climbers m_climbers;
  XboxController m_controller;
  double rspeed;
  double lspeed;
  public Climb2(Climbers climbers, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climbers = climbers;
    m_controller = controller;
    addRequirements(m_climbers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rspeed = 0.0;
    lspeed = 0.0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_controller.getRightBumper()){

      lspeed = 0.5;
      rspeed = -0.5;
    }

    if (m_controller.getAButton()){
      //up (works)
      lspeed = -0.5;
      rspeed =  0.5;
    } else {
      lspeed = 0.0;
      rspeed = 0.0;
    }

    m_climbers.climberLeftMove(lspeed);
    m_climbers.climberRightMove(rspeed);
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
