// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climbers;

public class ClimbCommand extends Command {

  public final double DEADBAND = 0.15;

  /** Creates a new Climbers. */
  Climbers m_climbers;
  XboxController m_controller;

  public ClimbCommand(Climbers climber, XboxController controller ) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climbers = climber;
    m_controller = controller;
    addRequirements(m_climbers);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double lspeed = 0.0;
    double rspeed = 0.0;
  
    rspeed = m_controller.getRightY();
    m_climbers.climberRightMove(stick2speed(rspeed));

    lspeed = m_controller.getLeftY();
    m_climbers.climberLeftMove(stick2speed(lspeed));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbers.climberLeftMove(0.0);
    m_climbers.climberRightMove(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // applies deadband and scaling to raw stick value
  private double stick2speed(double stickValue) {
    return Math.signum(stickValue) * Math.pow(MathUtil.applyDeadband(stickValue, DEADBAND), 2);
  }
}
