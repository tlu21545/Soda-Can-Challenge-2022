// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DefaultAutoGroup extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public DefaultAutoGroup(Drivetrain drivetrain) {
    addCommands(
        new DriveDistance(0.5, 30, drivetrain),
        new TurnDegrees(0.5, 160, drivetrain),
        new WaitCommand(0.1),
        new DriveDistance(0.5, 30, drivetrain),
        new TurnDegrees(0.5, 160, drivetrain));
  }
}
