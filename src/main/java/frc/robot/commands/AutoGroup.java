// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoGroup extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutoGroup(Drivetrain drivetrain) {
    addCommands(
        new DriveTurnGyro(0.5, 10, drivetrain),
        new WaitCommand(0.1),
        new DriveDistance(0.5, 10, drivetrain),
        new WaitCommand(0.1),
        new DriveTurnGyro(-0.5, 20, drivetrain),
        new WaitCommand(0.1),
        new DriveDistance(0.5, 15, drivetrain),
        new WaitCommand(0.1),
        new DriveTurnGyro(0.5, 20, drivetrain),
        new WaitCommand(0.1),
        new DriveDistance(0.5, 15, drivetrain),
        new WaitCommand(0.1),
        new DriveTurnGyro(-0.5, 100, drivetrain),
        new WaitCommand(0.1),
        new DriveDistance(0.5, 10, drivetrain),
        new WaitCommand(0.1),
        new DriveTurnGyro(-0.5, 100, drivetrain),
        new WaitCommand(0.1),
        new DriveDistance(0.5, 15, drivetrain),
        new WaitCommand(0.1),
        new DriveTurnGyro(0.5, 20, drivetrain),
        new WaitCommand(0.1),
        new DriveDistance(0.5, 15, drivetrain),
        new WaitCommand(0.1),
        new DriveTurnGyro(-0.5, 20, drivetrain),
        new WaitCommand(0.1),
        new DriveDistance(0.5, 10, drivetrain));
  }
}
