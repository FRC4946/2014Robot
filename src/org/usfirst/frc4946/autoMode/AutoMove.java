/*
 * Autonumous routine:
 *    1. Move forwards, until 12
 */
package org.usfirst.frc4946.autoMode;

import edu.wpi.first.wpilibj.RobotDrive;
import org.usfirst.frc4946.DistanceSensor;
import org.usfirst.frc4946.IntakeArm;
import org.usfirst.frc4946.Launcher;
import org.usfirst.frc4946.Loader;
import org.usfirst.frc4946.RobotConstants;
import edu.wpi.first.wpilibj.Gyro;

/**
 *
 * @author Stefan
 */
public class AutoMove extends AutoMode {
    
    public AutoMove(RobotDrive drive, Launcher launcher, Loader loader, IntakeArm intakeArm, DistanceSensor distanceSensor, Gyro gyro) {
        super(drive, launcher, loader, intakeArm, distanceSensor, gyro);
    }

    public void init() {
        resetGyro();
    }

    public void run() {
        driveToDistance(5*12, 0.4);
        m_driverStation.println(RobotConstants.AUTO_LCD_DRIVER, 1, "Driving to 12 ft at 0.4        ");
        
        if (atDistance(5*12)) {
            //celebrate we drove distance
            m_driverStation.println(RobotConstants.AUTO_LCD_DRIVER, 1, "AT DIST 12ft                  ");
            extendLoader();
            //do donuts
            //drive(0, 0.5);
        }
    }
}
