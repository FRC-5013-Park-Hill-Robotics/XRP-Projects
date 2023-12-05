// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SensorGroup;

public class Roomba extends Command {
  private final Drivetrain m_drive;
  private final double m_distance;
  private double m_traveled_distance;
  private final double m_speed;
  private final AnalogInput m_input;
  private final double m_bar;
  private final double m_turn_degrees;
  private int state;

  /**
   * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param inches The number of inches the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
  public Roomba(double speed, double inches, Drivetrain drive, SensorGroup input, double bar, double turn) {
    m_distance = inches;
    m_traveled_distance = 0;
    m_speed = speed;
    m_drive = drive;
    m_input = input.m_rangeFinder;
    m_bar = bar;
    m_turn_degrees = turn;
    state = 0;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(state == 0){
      if(m_input.getAverageVoltage() > m_bar){
       m_drive.arcadeDrive(m_speed, 0);
      } else {
       state = 1;
       m_traveled_distance += Math.abs(m_drive.getAverageDistanceInch());
       initialize();
      }
    }
    else if(state == 1){
      m_drive.arcadeDrive(0, m_speed);
      if(turnIsFinished()){
        state = 0;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to desired distance
    return m_traveled_distance >= m_distance;
  }

  public boolean turnIsFinished() {
    double inchPerDegree = Math.PI * 6.102 / 360;
    // Compare distance travelled from start to distance based on degree turn
    return getAverageTurningDistance() >= (inchPerDegree * m_turn_degrees);
  }

  private double getAverageTurningDistance() {
    double leftDistance = Math.abs(m_drive.getLeftDistanceInch());
    double rightDistance = Math.abs(m_drive.getRightDistanceInch());
    return (leftDistance + rightDistance) / 2.0;
  }
}
