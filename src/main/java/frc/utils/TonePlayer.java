// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
interface TonePlayerIF {

    public void setTones(double ... tones);
    public void stop();
} 

public abstract class TonePlayer extends SubsystemBase implements TonePlayerIF {

}