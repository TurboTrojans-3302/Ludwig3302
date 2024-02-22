// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class TranslateCommand extends Command {

  private final double DISTANCE_TOLERANCE = 0.050;

  private Pose2d m_dest;
  private Translation2d m_delta;
  private DriveSubsystem m_drive;

  /** Creates a new Translate. */
  public TranslateCommand(DriveSubsystem drive, Translation2d delta){
    m_drive = drive;
    m_delta = delta;
    addRequirements(m_drive);
  }

  public TranslateCommand(DriveSubsystem drive, double x, double y){
    this(drive, new Translation2d(x, y));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d start = m_drive.getPose();
    m_dest = new Pose2d(start.getTranslation().plus(m_delta),
                        start.getRotation());
  }

  private Translation2d translation2dest(){
    return m_dest.minus(m_drive.getPose()).getTranslation();
  }

  private double distance(){
    return translation2dest().getNorm();
  }

  private double calculateSpeed(){
    return Constants.DriveConstants.kMaxSpeedMetersPerSecond / 5.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = calculateSpeed();
    Translation2d unitTranslation = translation2dest().div(translation2dest().getNorm());
    m_drive.drive(unitTranslation.times(speed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setAll(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distance() < DISTANCE_TOLERANCE;
  }
}
