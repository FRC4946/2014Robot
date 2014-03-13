/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc4946;

import org.usfirst.frc4946.autoMode.*;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.camera.AxisCamera;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class BetaRobot extends SimpleRobot {

    RobotDrive m_robotDrive = new RobotDrive(RobotConstants.PWM_MOTOR_LEFT_FRONT, RobotConstants.PWM_MOTOR_LEFT_REAR, RobotConstants.PWM_MOTOR_RIGHT_FRONT, RobotConstants.PWM_MOTOR_RIGHT_REAR);

    IntakeArm m_intakeArm = new IntakeArm();
    Launcher m_launcher = new Launcher();
    Loader m_loader = new Loader();
    DistanceSensor m_distanceSensor = new DistanceSensor();

    Joystick m_driveJoystick = new Joystick(RobotConstants.JOYSTICK_LEFT);
    Joystick m_taskJoystick = new Joystick(RobotConstants.JOYSTICK_RIGHT);

    DriverStationLCD m_driverStation = DriverStationLCD.getInstance();

    Timer m_timer = new Timer();
    
    AxisCamera camera;
    
    // The first arg is the pressure switch, which will open at 115 psi and reclose at 95. It's value will be used to activate and deactivate the relay.
    // The second is the compressor's relay (The Spike module). It is what turns on and off the compressor.
    Compressor m_primaryCompressor = new Compressor(RobotConstants.COMPRESSOR_PRESSURE_SWITCH, RobotConstants.COMPRESSOR_RELAY);

    boolean buttonIntakeRollerIsDown = false;
    boolean intakeIsRear = false;
    public boolean modeRPM = true;

    double launcherSpeed = 0.0;
    double oldLauncherSpeed = 0.0;
    boolean speedIsPreset = false;

    //This function called once at system start.
    protected void robotInit() {
        // Start the compressor, let it do it's thing. It will turn on and off automatically to regulate pressure.
        m_primaryCompressor.start();
        camera = AxisCamera.getInstance();
    }

    /**
     * Our constructor for the BetaRobot class.
     */
    public BetaRobot() {

        // Set the speed of the motors on the launcher and intake
        //m_intakeArm.setSpeedOpenLoop(12);
        m_launcher.setSpeedOpenLoop(3);

    }

    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() {

        m_driverStation.println(RobotConstants.LCD_MODE_MISC, 1, "Entering autonomous                    ");
        m_driverStation.updateLCD();

        //AutoTwoBall m_routine = new AutoTwoBall(m_robotDrive, m_launcher, m_loader, m_intakeArm, m_distanceSensor);
        //AutoShootAndDrive m_routine = new AutoShootAndDrive(m_robotDrive,m_launcher,m_loader,m_intakeArm,m_distanceSensor);
        AutoVision_MoveAndShoot m_routine = new AutoVision_MoveAndShoot(m_robotDrive,m_launcher,m_loader,m_intakeArm,m_distanceSensor);
        //AutoMove m_routine = new AutoMove(m_robotDrive,m_launcher,m_loader,m_intakeArm,m_distanceSensor);
        int m_cycleNumber = 0;
        m_routine.init();
        
        m_routine.giveCamera(camera);
        
        while (isAutonomous() && isEnabled()) {

            m_cycleNumber++;
            m_routine.run();
            
            if ((m_cycleNumber % RobotConstants.CONSOLE_UPDATE_TIME) == 0) {

                m_driverStation.updateLCD();
                m_cycleNumber = 0;
            }
            Timer.delay(0.001);
        }
        m_driverStation.println(RobotConstants.LCD_MODE_MISC, 1, "Finished auto, waiting                   ");
        m_driverStation.updateLCD();

    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {

        m_driverStation.println(RobotConstants.LCD_MODE_MISC, 1, "Entering operator control                 ");
        m_driverStation.updateLCD();

        int m_cycleNumber = 0;

        while (isOperatorControl() && isEnabled()) {

            m_cycleNumber++;

            //Call the drive oriented code
            //operatorDriveSystem();

            //Call the task oriented code
            //operatorTaskSystem();

            m_driverStation.println(RobotConstants.LCD_RANGE, 1, "Range: " + m_distanceSensor.getRangeInchs() + "\"            ");

            if ((m_cycleNumber % RobotConstants.CONSOLE_UPDATE_TIME) == 0) {

                m_driverStation.updateLCD();
                m_cycleNumber = 0;
            }
            
            Timer.delay(0.001);
        }

        m_driverStation.println(RobotConstants.LCD_MODE_MISC, 1, "Stopping operator control           ");
        m_driverStation.updateLCD();

    }

    /**
     * This function contains code pertaining to the task joystick. It is called
     * once every loop of the operator control cycle.
     */
    private void operatorTaskSystem() {
        //********* INTAKE *********\\
        // Extend and turn on the intake arm
        if (m_taskJoystick.getRawButton(RobotConstants.JOYSTICK_BUTTON_INTAKE_OUT)) {

            m_intakeArm.setExtended(true);

        }

        // Retract and turn off the intake arm
        if (m_taskJoystick.getRawButton(RobotConstants.JOYSTICK_BUTTON_INTAKE_IN)) {

            m_intakeArm.setExtended(false);
        }

        m_intakeArm.updateSolenoids();

        // If the intake roller button is pressed, get ready for its release
        if (m_taskJoystick.getRawButton(RobotConstants.JOYSTICK_BUTTON_INTAKE_ROLLER)) {
            buttonIntakeRollerIsDown = true;
        }

        // If the intake roller button is released, toggle the state of the solenoid
        if (!m_taskJoystick.getRawButton(RobotConstants.JOYSTICK_BUTTON_INTAKE_ROLLER)
                && buttonIntakeRollerIsDown == true) {

            buttonIntakeRollerIsDown = false;
            
                m_intakeArm.toggleEnabled();
            
        }

        //********* LOADER *********\\
        // If the trigger is down, lift the ball into the rollers		
        m_loader.setExtended(m_driveJoystick.getTrigger());
        m_loader.updateSolenoids();

        //********* LAUNCHER *********\\
        // If the launch button is pressed, get ready for its release
        if (m_taskJoystick.getRawButton(RobotConstants.JOYSTICK_BUTTON_LAUNCHER_ON)) {
            if (modeRPM) {
                m_launcher.setOpenLoopEnabled(true);
            } else {
                m_launcher.setClosedLoopEnabled(true);
            }
            m_intakeArm.setExtended(true); // Extend the intake arm, just in case
        }

        // If the launch button is released, toggle the state of the solenoid
        if (m_taskJoystick.getRawButton(RobotConstants.JOYSTICK_BUTTON_LAUNCHER_OFF)) {
            if (modeRPM) {
                m_launcher.setOpenLoopEnabled(false);
            } else {
                m_launcher.setClosedLoopEnabled(false);
            }
        }
        // Decide whether to used open or closed loop control
        if (m_taskJoystick.getRawButton(RobotConstants.JOYSTICK_BUTTON_RPM_MODE)) {
            modeRPM = true;
        } else if (m_taskJoystick.getRawButton(RobotConstants.JOYSTICK_BUTTON_VOLTAGE_MODE)) {
            modeRPM = false;

        }

        // If the slider has been changed, disable the preset
        if (speedIsPreset && m_taskJoystick.getZ() != oldLauncherSpeed) {
            speedIsPreset = false;
        }

        // Set the speed to the presets
        if (m_taskJoystick.getRawButton(RobotConstants.JOYSTICK_BUTTON_LAUNCHER_PRESET_ONE)) {
            modeRPM = true;
            launcherSpeed = 1400;
            speedIsPreset = true;
        } else if (m_taskJoystick.getRawButton(RobotConstants.JOYSTICK_BUTTON_LAUNCHER_PRESET_TWO)) {
            modeRPM = true;
            launcherSpeed = 1500;
            speedIsPreset = true;
           // m_intakeArm.setEnabledRollersReverse(true);
        } else if (m_taskJoystick.getRawButton(RobotConstants.JOYSTICK_BUTTON_LAUNCHER_PRESET_THREE)) {
            modeRPM = true;
            //launcherSpeed = 1750;
            launcherSpeed = 3000;
            speedIsPreset = true;
        }

        // If the speed was not last set using the preset buttons, read from the slider
        if (!speedIsPreset) {
            // Set the launcher speed to the Z val, and then update the motors
            launcherSpeed = m_taskJoystick.getZ();
            oldLauncherSpeed = launcherSpeed;
        }

        
        // If the trigger is down, brake. Otherwise, set the speed of the motors normally.
        if (m_taskJoystick.getTrigger()) {
            m_launcher.setSpeedOpenLoop(0.5);
            m_launcher.setOpenLoopEnabled(m_launcher.isEnabled());
            m_driverStation.println(RobotConstants.LCD_DRIVER, 1, "Braking...                   ");
        } else {
            // Open loop  -  Set the speed with voltage
            if (modeRPM == false) {
                launcherSpeed *= -1;                        //Flip range from (1, -1) to (-1, 1)
                launcherSpeed = (launcherSpeed + 1) / 2;    // Shift to (0,1)

                launcherSpeed *= RobotConstants.SHOOTER_MAX_VOLTAGE;                       // Scale to max voltage set in constants
                m_launcher.setSpeedOpenLoop(launcherSpeed);
                m_launcher.setOpenLoopEnabled(m_launcher.isEnabled());
                m_driverStation.println(RobotConstants.LCD_DRIVER, 1, "OpenSpeed: " + launcherSpeed + "                 ");

            } else if (modeRPM == true) {
                // Closed loop  -  Set the speed with a RPM
                if (!speedIsPreset) {
                    launcherSpeed *= -1;                        //Flip range from (1, -1) to (-1, 1)
                    launcherSpeed = 1850 + (launcherSpeed * 500);
                }

                m_launcher.setSpeedRPM(launcherSpeed);
                m_launcher.setClosedLoopEnabled(m_launcher.isEnabled());
                m_driverStation.println(RobotConstants.LCD_DRIVER, 1, "ClosedSpeed: " + launcherSpeed + "                 ");

            }
        }
        m_launcher.update();

    }

    /**
     * This function contains code pertaining to the drive joystick. It is
     * called once every loop of the operator control cycle.
     */
    private void operatorDriveSystem() {

        // Set the launcher speed to the Z val, and then update the motors
        double driveSpeed = m_driveJoystick.getThrottle();

        driveSpeed *= -1;                        //Flip range from (1, -1) to (-1, 1)
        driveSpeed = (driveSpeed + 1) / 2;    // Shift to (0,1)

        //Drive the robot with either better turning, or maximum speed depending on the trigger.
        double outputMagnitude = m_driveJoystick.getY();
        double curve = m_driveJoystick.getX();

        outputMagnitude = outputMagnitude * (0.5 + 0.5 * driveSpeed); // 0.5 to 1.0
        curve = (curve * (0.7 + 0.2 * driveSpeed)) + 0.001; // 0.7 to 0.9

        
        m_driverStation.println(RobotConstants.LCD_LOADER, 1, "Curve: "+curve+"                      ");
        
//        if (m_driveJoystick.getTrigger()) {
//            //allow fast speed, but reduce turning
//            outputMagnitude *= 1.0;
//            curve *= 0.8;
//
//        } else {
//            //Drive slow if the trigger is not down
//            outputMagnitude *= 0.6;
//            curve *= 0.7;
//
//        }
        // Set the orientation (which way is front)
        if (m_driveJoystick.getRawButton(RobotConstants.JOYSTICK_BUTTON_SHOOT_ORIENTATION)) {
            intakeIsRear = true;
        } else if (m_driveJoystick.getRawButton(RobotConstants.JOYSTICK_BUTTON_INTAKE_ORIENTATION)) {
            intakeIsRear = false;
        }

        // Call arcadeDrive with the updated orientation
        if (intakeIsRear) {
            m_robotDrive.arcadeDrive(outputMagnitude, curve * -1, true);
        } else {
            m_robotDrive.arcadeDrive(outputMagnitude * -1, curve * -1, true);
        }
    }
}
