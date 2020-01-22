/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
          MotorLocation.FRONT_LEFT, new Translation2d(0.25, -0.25),
          MotorLocation.FRONT_RIGHT, new Translation2d(0.25, 0.25),
          MotorLocation.BACK_LEFT, new Translation2d(-0.25, -0.25),
          MotorLocation.BACK_RIGHT, new Translation2d(-0.25, 0.25)
  );

  private final Map<MotorLocation, WPI_VictorSPX> driveMotors = Map.of(
          MotorLocation.FRONT_LEFT, new WPI_VictorSPX(Constants.frontLeftDriveMotorNumber),
          MotorLocation.FRONT_RIGHT, new WPI_VictorSPX(Constants.frontRightDriveMotorNumber),
          MotorLocation.BACK_LEFT, new WPI_VictorSPX(Constants.backLeftDriveMotorNumber),
          MotorLocation.BACK_RIGHT, new WPI_VictorSPX(Constants.backRightDriveMotorNumber)
  );

  private final Map<MotorLocation, WPI_TalonSRX> steeringMotors = Map.of(
          MotorLocation.FRONT_LEFT, new WPI_TalonSRX(Constants.frontLeftSteeringMotorNumber),
          MotorLocation.FRONT_RIGHT, new WPI_TalonSRX(Constants.frontRightSteeringMotorNumber),
          MotorLocation.BACK_LEFT, new WPI_TalonSRX(Constants.backLeftSteeringMotorNumber),
          MotorLocation.BACK_RIGHT, new WPI_TalonSRX(Constants.backRightSteeringMotorNumber)
  );

  private Map<MotorLocation, Boolean> driveMotorInverted = Map.of(
          MotorLocation.FRONT_LEFT, false,
          MotorLocation.FRONT_RIGHT, false,
          MotorLocation.BACK_LEFT, false,
          MotorLocation.BACK_RIGHT, false
  );

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    //Configure the Talons for PID control
    for (Drivetrain.MotorLocation loc : Drivetrain.MotorLocation.values()) {
      WPI_TalonSRX talon = steeringMotors.get(loc);
      talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
      talon.config_kP(0, 10);
      talon.config_kD(0, 0);
      talon.config_kI(0, 0);
      talon.configAllowableClosedloopError(0, 5);
      talon.configClosedloopRamp(0);
    }

    steeringMotors.get(MotorLocation.FRONT_LEFT).setInverted(false);
    steeringMotors.get(MotorLocation.FRONT_RIGHT).setInverted(true);
    steeringMotors.get(MotorLocation.BACK_LEFT).setInverted(false);
    steeringMotors.get(MotorLocation.BACK_RIGHT).setInverted(true);
  }

  @Override
  public void periodic() {
  }

  /**
   * Sets the speed of an individual drive motor
   * @param motor the motor to control
   * @param speed the desired motor speed, from -1 to 1
   */
  public void setDriveMotorSpeed(MotorLocation motor, double speed) {
    if (driveMotorInverted.get(motor)) {
      speed = -speed;
    }
    driveMotors.get(motor).set(speed);
  }

  /**
   * Sets the speeds of the drive motors such that the robot moves as specified by the forward and rotation paramater
   * Assumes that the wheels are pointed forwards
   * @param forward The desired forwards speed, from -1 to 1
   * @param rotation The desired angular velocity, from -1 to 1
   */
  public void arcadeDrive(double forward, double rotation) {
    double leftSpeed = forward - rotation;
    double rightSpeed = forward + rotation;

    double max = Math.max(1, Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed)));
    leftSpeed /= max;
    rightSpeed /= max;

    setDriveMotorSpeed(MotorLocation.FRONT_LEFT, leftSpeed);
    setDriveMotorSpeed(MotorLocation.BACK_LEFT, leftSpeed);
    setDriveMotorSpeed(MotorLocation.FRONT_RIGHT, rightSpeed);
    setDriveMotorSpeed(MotorLocation.BACK_RIGHT, rightSpeed);
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

    for (Drivetrain.MotorLocation loc : Drivetrain.MotorLocation.values()) {
      steeringMotors.get(loc).set(ControlMode.PercentOutput, 0);
    }
  }

  /**
   * Find the displacement between the two angles with the smallest magnitude, assuming 0 degrees is equal to 360 degrees
   * @param angle1 The value of the initial angle, in degrees
   * @param angle2 THe value of the final angle, in degrees
   * @return The calculted displacement, in degrees
   */
  public double getAngleDifference(double angle1, double angle2) {
    if (Math.abs(angle2 - angle1) <= 360 - Math.abs(angle2 - angle1)) {
      return angle2 - angle1;
    } else {
      return Math.copySign(360 - Math.abs(angle1 - angle2), angle1 - angle2);
    }
  }

  /**
   * Calculate the angle of one of the swerve wheels based on its encoder position
   * @param wheel The wheel to find the angle of
   * @return The calculated angle, in degrees, counterclockwise from the initial position, from 0 to 360
   */
  public double getWheelAngle(MotorLocation wheel) {
    WPI_TalonSRX talon = steeringMotors.get(wheel);

    //Get the position of the encoder, in raw units
    int encoderTicks = talon.getSelectedSensorPosition();

    //Convert the encoder position to a wheel angle
    double angle =  (encoderTicks / Constants.pulsesPerRevolution * 360) % 360;
    if (angle < 0) {angle += 360;}
    return angle;
  }

  /**
   * Instruct the talon's integrated PID loop to rotate the wheel to a specific angle
   * @param wheel The wheel to control
   * @param target_angle The desired angle, in degrees, clockwise from the initial position, from 0 to 360
   */
  public void setDesiredWheelAngle(MotorLocation wheel, double target_angle) {
    target_angle = target_angle % 360;
    if (target_angle < 0) {target_angle += 360;}

    WPI_TalonSRX talon = steeringMotors.get(wheel);
    double current_angle = getWheelAngle(wheel);
    int current_encoder_position = talon.getSelectedSensorPosition();

    double displacement = getAngleDifference(current_angle, target_angle);
    if (Math.abs(displacement) < 90) {
      driveMotorInverted.put(wheel, false);
    } else {
      driveMotorInverted.put(wheel, true);
      target_angle = (target_angle + 180) % 360;
      displacement = getAngleDifference(current_angle, target_angle);
    }

    double desiredEncoderPosition = current_encoder_position + displacement * Constants.pulsesPerRevolution / 360.0;
    talon.set(ControlMode.Position, desiredEncoderPosition);
  }
}