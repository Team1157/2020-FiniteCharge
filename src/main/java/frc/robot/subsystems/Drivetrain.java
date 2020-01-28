/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX; //Note: add build/libs/FRC 2020 to your classpath.
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
    private final double STEERING_ANGLE_TOLERANCE = 3; //In degrees

    public enum MotorLocation {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }

    //Stores the location of each drive wheel, in meters. Important for swerve drive math.
    public static final Map<MotorLocation, Translation2d> wheelCoordinates = Map.of( //TODO
            MotorLocation.FRONT_LEFT, new Translation2d(0.25, -0.25),
            MotorLocation.FRONT_RIGHT, new Translation2d(0.25, 0.25),
            MotorLocation.BACK_LEFT, new Translation2d(-0.25, -0.25),
            MotorLocation.BACK_RIGHT, new Translation2d(-0.25, 0.25)
    );

    private static final Map<MotorLocation, Integer> WHEEL_ENCODER_ZERO_POSITION = Map.of( //TODO
            MotorLocation.FRONT_LEFT, 308,
            MotorLocation.FRONT_RIGHT, 415,
            MotorLocation.BACK_LEFT, 390,
            MotorLocation.BACK_RIGHT, 433
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

    private final Map<MotorLocation, Encoder> driveMotorEncoders = Map.of(
            MotorLocation.FRONT_LEFT, new Encoder(0, 1),
            MotorLocation.FRONT_RIGHT, new Encoder(2, 3),
            MotorLocation.BACK_LEFT, new Encoder(4, 5),
            MotorLocation.BACK_RIGHT, new Encoder(6, 7)
    );

    private HashMap<MotorLocation, Boolean> driveMotorInverted = new HashMap<>();


    private final double PID_P_GAIN = 0; //TODO Tune controller
    private final double PID_D_GAIN = 0;
    private final double PID_I_GAIN = 0;
    private final double PID_TOLERANCE = 1; //In degrees
    private final PIDController rotationPIDController;

    /**
     * Creates a new Drivetrain.
     */
    public Drivetrain() {
        //Configure the Talons for PID control
        for (Drivetrain.MotorLocation loc : Drivetrain.MotorLocation.values()) {
            driveMotorInverted.put(loc, false);

            WPI_TalonSRX talon = steeringMotors.get(loc);
            talon.configFactoryDefault();
            talon.configSelectedFeedbackSensor(FeedbackDevice.Analog);
            talon.config_kP(0, 4);
            talon.config_kD(0, 8);
            talon.config_kI(0, 0);
            talon.configAllowableClosedloopError(0, (int) (STEERING_ANGLE_TOLERANCE / 360.0 * Constants.steeringEncoderPulsesPerRevolution));
            talon.configFeedbackNotContinuous(true, 0);
        }

        steeringMotors.get(MotorLocation.FRONT_LEFT).setInverted(false);
        steeringMotors.get(MotorLocation.FRONT_RIGHT).setInverted(false);
        steeringMotors.get(MotorLocation.BACK_LEFT).setInverted(false);
        steeringMotors.get(MotorLocation.BACK_RIGHT).setInverted(false);

        rotationPIDController = new PIDController(
            PID_P_GAIN,
            PID_I_GAIN,
            PID_D_GAIN
        );
        rotationPIDController.setTolerance(PID_TOLERANCE);
        rotationPIDController.enableContinuousInput(0, 360);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("FL Angle", getWheelAngle(MotorLocation.FRONT_LEFT));
        SmartDashboard.putNumber("FL Encoder",  steeringMotors.get(MotorLocation.FRONT_LEFT).getSelectedSensorPosition());
        SmartDashboard.putNumber("FR Angle", getWheelAngle(MotorLocation.FRONT_RIGHT));
        SmartDashboard.putNumber("FR Encoder",  steeringMotors.get(MotorLocation.FRONT_RIGHT).getSelectedSensorPosition());
        SmartDashboard.putNumber("BL Angle", getWheelAngle(MotorLocation.BACK_LEFT));
        SmartDashboard.putNumber("BL Encoder",  steeringMotors.get(MotorLocation.BACK_LEFT).getSelectedSensorPosition());
        SmartDashboard.putNumber("BR Angle", getWheelAngle(MotorLocation.BACK_RIGHT));
        SmartDashboard.putNumber("BR Encoder",  steeringMotors.get(MotorLocation.BACK_RIGHT).getSelectedSensorPosition());

        SmartDashboard.putNumber("FR Drive Encoder", driveMotorEncoders.get(MotorLocation.BACK_RIGHT).get());
    }

    /**
     * Set the speed of an individual drive motor
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
     * Set the speeds of the drive motors such that the robot moves as specified by the forward and rotation paramater
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
    private double getAngleDifference(double angle1, double angle2) {
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
        int encoderTicks = Math.min(Math.max(talon.getSelectedSensorPosition(), Constants.steeringEncoderMin), Constants.steeringEncoderMax);

        //Convert the encoder position to a wheel angle
        double angle =  ((encoderTicks - WHEEL_ENCODER_ZERO_POSITION.get(wheel)) / Constants.steeringEncoderPulsesPerRevolution * 360) % 360;
        if (angle < 0) {angle += 360;}
        return angle;
    }

    /**
     * Determine whether the specified wheel angle is within the tolerance STEERING_ANGLE_TOLERANCE
     *
     * @param wheel The wheel to check
     * @return Whether the wheel is within tolerance
     */
    private boolean isWheelWithinTolerance(MotorLocation wheel) {
        return steeringMotors.get(wheel).getClosedLoopError() < (int) (STEERING_ANGLE_TOLERANCE / 360.0 * Constants.steeringEncoderPulsesPerRevolution);
    }

    /**
     * Determine whether all of the wheels are within the tolerance STEERING_ANGLE_TOLERANCE
     *
     * @return Whether the wheels are within tolerance
     */
    public boolean areAllWheelsWithinTolerance() {
        return (
            isWheelWithinTolerance(MotorLocation.FRONT_LEFT) &&
            isWheelWithinTolerance(MotorLocation.FRONT_RIGHT) &&
            isWheelWithinTolerance(MotorLocation.BACK_LEFT) &&
            isWheelWithinTolerance(MotorLocation.BACK_RIGHT)
        );
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
        double displacement = getAngleDifference(current_angle, target_angle);

        /*
        if (Math.abs(displacement) < 90) {
            driveMotorInverted.put(wheel, false);
        } else {
            driveMotorInverted.put(wheel, true);
            target_angle = (target_angle + 180) % 360;
            displacement = getAngleDifference(current_angle, target_angle);
        }

         */

        int current_encoder_position = talon.getSelectedSensorPosition();
        double desiredEncoderPosition = current_encoder_position + displacement * Constants.steeringEncoderPulsesPerRevolution / 360.0;
        talon.set(ControlMode.Position, desiredEncoderPosition);
    }

    /**
     * Angle the wheels perpendicular to the line connecting them to the center of the chassis, ideal for rotation in place
     */
    private void angleWheelsForRotation() {
        SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
                wheelCoordinates.get(MotorLocation.FRONT_LEFT),
                wheelCoordinates.get(MotorLocation.FRONT_RIGHT),
                wheelCoordinates.get(MotorLocation.BACK_LEFT),
                wheelCoordinates.get(MotorLocation.BACK_RIGHT)
        );

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, -1);
        SwerveModuleState[] moduleStatesList = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        Map<Drivetrain.MotorLocation, SwerveModuleState> moduleStates = Map.of(
                Drivetrain.MotorLocation.FRONT_LEFT, moduleStatesList[0],
                Drivetrain.MotorLocation.FRONT_RIGHT, moduleStatesList[1],
                Drivetrain.MotorLocation.BACK_LEFT, moduleStatesList[2],
                Drivetrain.MotorLocation.BACK_RIGHT, moduleStatesList[3]
        );

        for(Drivetrain.MotorLocation wheel : Drivetrain.MotorLocation.values()) {
            double rawAngle = moduleStates.get(wheel).angle.getDegrees();
            setDesiredWheelAngle(wheel, -rawAngle);
        }
    }

    /**
     * Initialize the pid controller to rotate towards a desired heading. You must call updateRotationPID() to update it.
     */
    public void initRotationPID() {
        rotationPIDController.reset();
        angleWheelsForRotation();
    }

    /**
     * Once the pid has been initialized, update the drive wheel speeds based on the current gyro angle
     *
     * @param currentAngle The current rotation of the robot, as measured by the gyro
     */
    public void updateRotationPID(double currentAngle) {
        currentAngle = currentAngle % 360;
        if (currentAngle < 0) { currentAngle += 360; }

        double rotationSpeed = rotationPIDController.calculate(currentAngle);
        for (MotorLocation motorLocation: MotorLocation.values()) {
            if (areAllWheelsWithinTolerance()) {
                setDriveMotorSpeed(motorLocation, rotationSpeed);
            } else {
                setDriveMotorSpeed(motorLocation, 0);
            }
        }
    }

    /**
     * Change the target angle of the rotation PID controller
     *
     * @param setpoint The new setpoint
     */
    public void setRotationPIDSetpoint(double setpoint) {
        rotationPIDController.setSetpoint(setpoint);
    }

    /**
     * Check whether the PID controller is within the tolerance PID_TOLERANCE
     *
     * @return whether the controller is within tolerance
     */
    public boolean isRotationPIDWithinTolerance() {
        return rotationPIDController.atSetpoint();
    }
}