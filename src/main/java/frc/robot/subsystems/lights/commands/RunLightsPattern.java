// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lights.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.lights.Lights;

public class RunLightsPattern extends Command {
  /** Creates a new RunLightsPattern. */
  private final String targetPattern;
  private final double[] color;
  private final double[][] colors;
  private final int durationInMillis;
  public RunLightsPattern(String targetPattern, double[] color, double[][] colors, int durationInMillis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetPattern = targetPattern;
    this.color = color;
    this.colors = colors;
    this.durationInMillis = durationInMillis;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (targetPattern == "makeLightsColors") {
      Lights.makeLightsColors(color);
    } else if (targetPattern == "solidColor") {
      Lights.solidColor(color);
    } else if (targetPattern == "colorCycle") {
      Lights.colorCycle(colors, durationInMillis);
    } else if (targetPattern == "colorFlash") {
      Lights.colorFlash(colors[0], colors[1], durationInMillis);
    } else if (targetPattern == "colorPulse") {
      Lights.colorPulse(color, durationInMillis);
    } else if (targetPattern == "colorFlicker") {
      Lights.colorFlicker(color, durationInMillis);
    } else if (targetPattern == "colorStrobe") {
      Lights.colorStrobe(color, durationInMillis);
    } else if (targetPattern == "colorFade") {
      Lights.colorFade(colors[0], colors[1], durationInMillis);
    } else if (targetPattern == "colorBurst") {
      Lights.colorBurst(colors[0], colors[1], durationInMillis);
    } else if (targetPattern == "colorSparkle") {
      Lights.colorSparkle(color, durationInMillis);
    } else if (targetPattern == "colorWave") {
      Lights.colorWave(colors, durationInMillis);
    } else if (targetPattern == "colorHueSparkle") {
      Lights.colorHueSparkle(color, durationInMillis);
    }

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
