/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionLights;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final Joystick primaryStick = new Joystick(0);
    private final Joystick secondaryStick = new Joystick(1);
    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    private final Drivetrain drivetrain = new Drivetrain();
    private final VisionLights visionLights = new VisionLights();

    private final SendableChooser<INPUT_MODE> inputModeChooser = new SendableChooser<>();
    private final ShuffleboardTab tab = Shuffleboard.getTab("Vision Debug");
    private NetworkTableEntry visionDebugChooser = tab.add("Vision Debug", false)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();

    private INPUT_MODE current_input_mode = INPUT_MODE.ONE_STICK;
    public enum INPUT_MODE {
        ONE_STICK,
        TWO_STICK
    }

    public static NetworkTable visionTable;
    public static NetworkTableInstance networkTableInstance;

    /**
     * Returns the current forward input from a joystick or other input device, based on the current control mode
     *
     * @return The input, from -1 to 1
     */
    public double getForwardInput() {
        switch (current_input_mode) {
            case ONE_STICK:
            case TWO_STICK:
                return -primaryStick.getY();
            default:
                return 0;
        }
    }

    /**
     * Returns the current right input from a joystick or other input device, based on the current control mode
     *
     * @return The input, from -1 to 1
     */
    public double getRightInput() {
        switch (current_input_mode) {
            case ONE_STICK:
            case TWO_STICK:
                return primaryStick.getX();
            default:
                return 0;
        }
    }

    /**
     * Returns the current right input from a joystick or other input device, based on the current control mode
     *
     * @return The input, from -1 to 1, with positive being counterclockwise
     */
    public double getRotationInput() {
        switch (current_input_mode) {
            case ONE_STICK:
                return -primaryStick.getZ();
            case TWO_STICK:
                return -secondaryStick.getX();
            default:
                return 0;
        }
    }

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        networkTableInstance = NetworkTableInstance.getDefault();
        visionTable = networkTableInstance.getTable("vision");

        inputModeChooser.setDefaultOption("One Stick", INPUT_MODE.ONE_STICK);
        inputModeChooser.addOption("Two Stick", INPUT_MODE.TWO_STICK);
        SmartDashboard.putData("Input Mode", inputModeChooser);

        SmartDashboard.putData("Swerve Test",
                new SwerveTest(
                        drivetrain,
                        this::getRightInput,
                        this::getForwardInput,
                        this::getRotationInput,
                        gyro::getAngle
                ));

        drivetrain.setDefaultCommand(
                //Allows the swerve drive command to access the joystick inputs
                new SwerveDrive(
                        drivetrain,
                        this::getRightInput,
                        this::getForwardInput,
                        this::getRotationInput,
                        gyro::getAngle
                ));
        // Configure the button bindings
        configureButtonBindings();

    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /*
        JoystickButton visionAlignButton = new JoystickButton(primaryStick, Constants.visionAlignButtonNumber);
        visionAlignButton.whileHeld(new VisionAlign(
                drivetrain,
                visionLights,
                gyro::getAngle
        ));
         */
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }

    void periodic() {
        current_input_mode = inputModeChooser.getSelected();
        visionTable.getEntry("VisionDebug").setBoolean(visionDebugChooser.getBoolean(false));

        SmartDashboard.putBoolean("Pi Frames Available", visionTable.getEntry("FramesAvailable").getBoolean(false));
        SmartDashboard.putBoolean("Pi Target Found", visionTable.getEntry("TargetFound").getBoolean(false));
    }
}
