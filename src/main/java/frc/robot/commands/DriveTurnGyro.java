// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class DriveTurnGyro extends CommandBase {
  private Drivetrain drivetrain;
  private double target; // how many degrees to turn between -180 and 180
  private double direction; // +1 = turn to the left, -1 = turn to the right
  private double maxVel;
  private double maxAccel;
  private long profileStartTime; // initial time of starting point
  private long currProfileTime;
  private double targetVel; // velocity to reach by end of profile, deg/sec (probably 0)
  private double targetAccel;
  private double startAngle, targetRel;
  private double currAngle, currVelocity, currVelocityGyro;
  private double timeSinceStart;
  private boolean regenerate;
  private PIDController pidAngVel;

  private final double kPAngular = DriveConstants.kPAngular;

  private TrapezoidProfile tProfile;
  private TrapezoidProfile.State tStateCurr; // initial state of the system (position/time)
  private TrapezoidProfile.State tStateNow;
  private TrapezoidProfile.State tStateForecast;
  private TrapezoidProfile.State tStateFinal;
  private TrapezoidProfile.Constraints tConstraints;

  public DriveTurnGyro(double target, double maxVel, double maxAccel, boolean regenerate, Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    this.target = target;
    this.maxVel = maxVel;
    this.maxAccel = maxAccel;
    this.regenerate = regenerate;
  
    addRequirements(drivetrain);

    pidAngVel = new PIDController(kPAngular, 0, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    startAngle = drivetrain.getGyroAngleX();
    targetRel = target;

    direction = Math.signum(targetRel);

    tStateFinal = new TrapezoidProfile.State(targetRel, 0.0);
    tStateCurr = new TrapezoidProfile.State(0.0, drivetrain.getAccelX());

    tConstraints = new TrapezoidProfile.Constraints(maxVel, maxAccel);
    tProfile = new TrapezoidProfile(tConstraints, tStateFinal, tStateCurr);

    profileStartTime = System.currentTimeMillis();
    currProfileTime = profileStartTime;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   double pFF, pFB, pDB;

   currProfileTime = System.currentTimeMillis();

   currAngle = drivetrain.getGyroAngleX() - startAngle;
   currVelocity = driveTrain.get
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /* Need to convert distance travelled to degrees. The Standard
       Romi Chassis found here, https://www.pololu.com/category/203/romi-chassis-kits,
       has a wheel placement diameter (149 mm) - width of the wheel (8 mm) = 141 mm
       or 5.551 inches. We then take into consideration the width of the tires.
    */
    double inchPerDegree = Math.PI * 5.551 / 360;
    // Compare distance travelled from start to distance based on degree turn
    return getAverageTurningDistance() >= (inchPerDegree * m_degrees);
  }

  private double getAverageTurningDistance() {
    double leftDistance = Math.abs(m_drive.getLeftDistanceInch());
    double rightDistance = Math.abs(m_drive.getRightDistanceInch());
    return (leftDistance + rightDistance) / 2.0;
  }
}
