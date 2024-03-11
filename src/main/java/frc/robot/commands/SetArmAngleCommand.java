// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Harvester;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetArmAngleCommand extends TrapezoidProfileCommand {
  /** Creates a new SetArmAngleCommand. */
  Harvester mHarvester;
  public SetArmAngleCommand(Harvester harvester, Double targetAngle) {
    super(
        // The motion profile to be executed
        new TrapezoidProfile(
            // The motion profile constraints
            new TrapezoidProfile.Constraints(Constants.harvesterConstants.MAX_ARM_SPEED,
                                             Constants.harvesterConstants.MAX_ARM_ACCEL)),
        state -> {
          harvester.setArmAngle(state.position);
        },
        // Goal state
        () -> new TrapezoidProfile.State(targetAngle, 0.0),
        // Current state
        () -> new TrapezoidProfile.State(harvester.getArmAngle(), harvester.getArmVelocity()),
        harvester);
  }
}
