/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX; //Note: add build/libs/FRC 2020 to your classpath.
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  public enum MotorLocation {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
  }

  //Stores the location of each drive wheel, in meters. Important for swerve drive math.
  public static final Map<MotorLocation, Translation2d> wheelCoordinates = Map.of(
          MotorLocation.FRONT_LEFT, new Translation2d(0, 0),
          MotorLocation.FRONT_RIGHT, new Translation2d(0, 0),
          MotorLocation.BACK_LEFT, new Translation2d(0, 0),
          MotorLocation.BACK_RIGHT, new Translation2d(0, 0)
  );

  private final Map<MotorLocation, WPI_VictorSPX> driveMotors =  Map.of(
          MotorLocation.FRONT_LEFT, new WPI_VictorSPX(Constants.frontLeftDriveMotorNumber),
          MotorLocation.FRONT_RIGHT, new WPI_VictorSPX(Constants.frontRightDriveMotorNumber),
          MotorLocation.BACK_LEFT, new WPI_VictorSPX(Constants.backLeftDriveMotorNumber),
          MotorLocation.BACK_RIGHT, new WPI_VictorSPX(Constants.backRightDriveMotorNumber)
  );

  /*
  private final Map<MotorLocation, WPI_TalonSRX> steeringMotors =  Map.of(
          MotorLocation.FRONT_LEFT, new WPI_TalonSRX(Constants.frontLeftSteeringMotorNumber),
          MotorLocation.FRONT_RIGHT, new WPI_TalonSRX(Constants.frontRightSteeringMotorNumber),
          MotorLocation.BACK_LEFT, new WPI_TalonSRX(Constants.backLeftSteeringMotorNumber),
          MotorLocation.BACK_RIGHT, new WPI_TalonSRX(Constants.backRightSteeringMotorNumber)
  );
   */

  private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(
          driveMotors.get(MotorLocation.FRONT_LEFT),
          driveMotors.get(MotorLocation.BACK_LEFT)
  );

  private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(
          driveMotors.get(MotorLocation.FRONT_RIGHT),
          driveMotors.get(MotorLocation.BACK_RIGHT)
  );

  //Differential drivetrain object used when driving in arcade mode
  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMotors, rightMotors);

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    //Configure the Talons for easy access to the encoders
    /*
    for (Drivetrain.MotorLocation loc : Drivetrain.MotorLocation.values()) {
      steeringMotors.get(loc).configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    }
     */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the speed of an individual drive motor
   * @param motor the motor to control
   * @param speed the desired motor speed, from -1 to 1
   */
  public void setDriveMotorSpeed(MotorLocation motor, double speed) {
    driveMotors.get(motor).set(speed);
  }

  /**
   * Sets the speeds of the drive motors such that the robot moves as specified by the forward and rotation paramater
   * Assumes that the wheels are pointed forwards
   * @param forward The desired forwards speed, from -1 to 1
   * @param rotation The desired angular velocity, from -1 to 1
   */
  public void arcadeDrive(double forward, double rotation) {
    differentialDrive.arcadeDrive(forward, rotation);
  }

  /**
   * Stop the drive motors immediately
   */
  public void stopDriveMotors() {
    for (Drivetrain.MotorLocation loc : Drivetrain.MotorLocation.values()) {
      setDriveMotorSpeed(loc, 0);
    }
  }

  /**
   * Stop all of the motors immediately
   */
  public void stopAll() {
    stopDriveMotors();
    /*
    for (Drivetrain.MotorLocation loc : Drivetrain.MotorLocation.values()) {
      steeringMotors.get(loc).set(0);
    }

     */
  }

  /**
   * Calculate the angle of one of the swerve wheels based on its encoder position
   * @param wheel The wheel to find the angle of
   * @return The calculated angle, in degrees, clockwise from the initial position, from 0 to 360
   */
  /*
  public double getWheelAngle(MotorLocation wheel) {
    WPI_TalonSRX talon = steeringMotors.get(wheel);

    //Get the position of the encoder, in raw units
    int encoderTicks = talon.getSelectedSensorPosition();

    //Convert the encoder position to a wheel angle
    return (encoderTicks / Constants.steeringEncoderPulsesPerRevolution / Constants.steeringGearRatio * 360) % 360;
  }
   */

  /**
   * Instruct the talon's integrated PID loop to rotate the wheel to a specific angle
   * @param wheel The wheel to control
   * @param angle The desired angle, in degrees, clockwise from the initial position, from 0 to 360
   */
  public void setDesiredWheelAngle(MotorLocation wheel, double angle) {
    //TODO Change the setpoint for the talon PID
  }
}