// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;

public class DriveTurnGyro extends CommandBase {
  private double turnTarget;
  private double turnSpeed;
  private Drivetrain drivetrain;

  private double kTurnP = DriveConstants.kTurnP;
  private double kTurnI = DriveConstants.kTurnI;
  private double kTurnD = DriveConstants.kTurnD;

  private double turnError; // sensor - desired
  private double turnPrevError = 0; // from last loop
  private double turnDerivative; // difference between error and prev error
  private double turnTotalError = 0; // Integral totalError = totalError + error

  public DriveTurnGyro(double speed, double target, Drivetrain drivetrain) {
    this.turnSpeed = speed;
    this.turnTarget = target;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.arcadeDrive(0, 0);
    drivetrain.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double turnCurr = drivetrain.getGyroAngleX();
    double turnSpeedPercent = 0;

    turnError = turnTarget - turnCurr;
    turnDerivative = turnError = turnPrevError;
    turnTotalError += turnError;
    turnSpeedPercent = 1 - ((turnError * kTurnP + turnDerivative * kTurnD + turnTotalError * kTurnI) / turnSpeed);

    drivetrain.arcadeDrive(0, turnSpeed * turnSpeedPercent);

    turnPrevError = turnError;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0);
    drivetrain.resetGyro();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetrain.getGyroAngleX() >= turnTarget;
  }
}
