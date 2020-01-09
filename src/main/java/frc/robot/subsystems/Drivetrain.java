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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX; //Note: add build/libs/FRC 2020 to your classpath.
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  enum motorLocation {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
  }

  private final Map<motorLocation, WPI_VictorSPX> driveMotors =  Map.of(
          motorLocation.FRONT_LEFT, new WPI_VictorSPX(Constants.frontLeftDriveMotorNumber),
          motorLocation.FRONT_RIGHT, new WPI_VictorSPX(Constants.frontRightDriveMotorNumber),
          motorLocation.BACK_LEFT, new WPI_VictorSPX(Constants.backLeftDriveMotorNumber),
          motorLocation.BACK_RIGHT, new WPI_VictorSPX(Constants.backRightDriveMotorNumber)
  );

  private final Map<motorLocation, WPI_TalonSRX> steeringMotors =  Map.of(
          motorLocation.FRONT_LEFT, new WPI_TalonSRX(Constants.frontLeftSteeringMotorNumber),
          motorLocation.FRONT_RIGHT, new WPI_TalonSRX(Constants.frontRightSteeringMotorNumber),
          motorLocation.BACK_LEFT, new WPI_TalonSRX(Constants.backLeftSteeringMotorNumber),
          motorLocation.BACK_RIGHT, new WPI_TalonSRX(Constants.backRightSteeringMotorNumber)
  );

  private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(
          driveMotors.get(motorLocation.FRONT_LEFT),
          driveMotors.get(motorLocation.BACK_LEFT)
  );

  private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(
          driveMotors.get(motorLocation.FRONT_RIGHT),
          driveMotors.get(motorLocation.BACK_RIGHT)
  );

  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMotors, rightMotors);

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {

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
  public void setDriveMotorSpeed(motorLocation motor, double speed) {
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
}