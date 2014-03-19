/*
 * Autonumous routine:
 *    1. Shoot the first ball
 *    2. Move forwards, until 8 feet away
 */

package org.usfirst.frc4946.autoMode;

import edu.wpi.first.wpilibj.Gyro;
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
public class AutoShootAndDrive extends AutoMode {

    //AutoMode autoRoutine = new AutoMode(m_robotDrive, m_launcher, m_loader, m_intakeArm, m_distanceSensor);
    int step = 0;
    int counter = 0;

    public AutoShootAndDrive(RobotDrive drive, Launcher launcher, Loader loader, IntakeArm intakeArm, DistanceSensor distanceSensor, Gyro gyro) {
        super(drive, launcher, loader, intakeArm, distanceSensor, gyro);
    }

    public void init() {
        
        resetGyro();
        
        step = 0;
        counter = 0;
        atDistanceCount = 0;

        extendArm();
        m_driverStation.println(RobotConstants.AUTO_LCD_INTAKE, 1, "EXTENDED ARM            ");
        m_driverStation.updateLCD();
        startShooter(true);
        setShooterSpeed(1750, true);
        m_driverStation.println(RobotConstants.AUTO_LCD_LAUNCHER, 1, "SHOOTER ON 1750               ");
    }
    int atDistanceCount = 0;

    public void run() {
        m_launcher.update();
        counter++;
        m_driverStation.println(RobotConstants.AUTO_LCD_INTAKE, 1, "Dist " + m_distanceSensor.getRangeInchs()+"                          ");
        if (step == 0 && shooterIsAtTargetSpeed(1750)) {
           m_loader.setExtended(true);
           step=1;
           counter=0;
           m_driverStation.println(RobotConstants.AUTO_LCD_LOADER, 1, "SHOOTING                       ");
        }

        if (step == 1 && counter>70) {
            m_loader.setExtended(false);
            step=2;
            counter=0;
        }
        
        if(step == 2 && counter>100){
            //driveToDistance(8 * 12, 0.4);
        }
        
        if (atDistance(8 * 12) && step == 2) {
            atDistanceCount++;
            if (atDistanceCount > 2) {
                drive(0.0, 0.0);
            }
        } else {
            atDistanceCount = 0;
        }
        
        if (atDistanceCount > 10 && step == 2) {
            step = 3;
            drive(0.0, 0.0);
            counter = 0;
            m_driverStation.println(RobotConstants.AUTO_LCD_LAUNCHER, 1, "AT DIS               ");
        }
    }

}
