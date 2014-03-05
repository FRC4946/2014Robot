/*
 * Autonumous routine:
 *    1. Move forwards, until 10 feet away
 *    2. Shoot the ball
 */

package org.usfirst.frc4946.autoMode;

import edu.wpi.first.wpilibj.RobotDrive;
import org.usfirst.frc4946.DistanceSensor;
import org.usfirst.frc4946.IntakeArm;
import org.usfirst.frc4946.Launcher;
import org.usfirst.frc4946.Loader;
import org.usfirst.frc4946.RobotConstants;

/**
 *
 * @author Stefan
 */
public class AutoMoveAndShoot extends AutoMode {

    //AutoMode autoRoutine = new AutoMode(m_robotDrive, m_launcher, m_loader, m_intakeArm, m_distanceSensor);
    int step = 0;
    int counter = 0;
    int atDistanceCount = 0;

    public AutoMoveAndShoot(RobotDrive drive, Launcher launcher, Loader loader, IntakeArm intakeArm, DistanceSensor distanceSensor) {
        super(drive, launcher, loader, intakeArm, distanceSensor);
    }

    public void init() {
        
        initGyroSensor();
        
        step = 0;
        counter = 0;
        atDistanceCount = 0;

        retractArm();

        startShooter(true);
        setShooterSpeed(1425, true);
        m_driverStation.println(RobotConstants.AUTO_LCD_LAUNCHER, 1, "SHOOTER ON 1425               ");
    }

    public void run() {
        m_launcher.update();
        counter++;
        m_driverStation.println(RobotConstants.AUTO_LCD_INTAKE, 1, "Dist " + m_distanceSensor.getRangeInchs()+"                          ");
        if (step == 0) {
            driveToDistance(9 * 12, 0.4);

        }

        if (atDistance(9 * 12) && step == 0) {
            atDistanceCount++;
            if (atDistanceCount > 2) {
                drive(0.0, 0.0);
            }
        } else {
            atDistanceCount = 0;

        }

        if (atDistanceCount > 10 && step == 0) {
            step = 1;
            drive(0.0, 0.0);
            counter = 0;
            m_driverStation.println(RobotConstants.AUTO_LCD_LAUNCHER, 1, "AT DIS & SPEED               ");
        }

        if (step == 1 && counter > 100) {

            extendLoader();
            m_driverStation.println(RobotConstants.AUTO_LCD_LOADER, 1, "SHOOTING                       ");
        }
    }

}
