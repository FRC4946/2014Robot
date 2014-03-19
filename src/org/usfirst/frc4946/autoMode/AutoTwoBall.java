/*
 * Autonumous routine:
 *    1. Shoot the first ball
 *    2. Pick up second car
 *    3. Shoot the second ball
 *    4. Move forwards, until 8 feet away
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
public class AutoTwoBall extends AutoMode {

    //AutoMode autoRoutine = new AutoMode(m_robotDrive, m_launcher, m_loader, m_intakeArm, m_distanceSensor);
    int step = 0;
    int counter = 0;

    public AutoTwoBall(RobotDrive drive, Launcher launcher, Loader loader, IntakeArm intakeArm, DistanceSensor distanceSensor, Gyro gyro) {
        super(drive, launcher, loader, intakeArm, distanceSensor, gyro);
    }

    public void init() {
        
        resetGyro();
        
        step = 0;
        counter = 0;
        atDistanceCount = 0;

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
           m_driverStation.println(RobotConstants.AUTO_LCD_LOADER, 1, "SHOOTING 1                       ");
        }

        if (step == 1 && counter>50) {
            m_loader.setExtended(false);
            m_intakeArm.setExtended(true);
            m_intakeArm.setEnabledRollers(true);
            step=2;
            counter=0;
        }
        
        if (step == 2 && shooterIsAtTargetSpeed(1750) && counter > 450) {
           //m_loader.setExtended(true);
           step=3;
           counter=0;
           m_driverStation.println(RobotConstants.AUTO_LCD_LOADER, 1, "SHOOTING 2                       ");
        }

        if (step == 3 && counter>50) {
            m_loader.setExtended(false);
            m_intakeArm.setEnabledRollers(false);
            step=4;
            counter=0;
        }
        
        if(step == 4 && counter>100){
            driveToDistance(8 * 12, 0.6);
        }
        
        if (atDistance(8 * 12) && step == 4) {
            atDistanceCount++;
            if (atDistanceCount > 2) {
                drive(0.0, 0.0);
            }
        } else {
            atDistanceCount = 0;
        }
        
        if (atDistanceCount > 10 && step == 4) {
            step = 5;
            drive(0.0, 0.0);
            counter = 0;
            m_driverStation.println(RobotConstants.AUTO_LCD_LAUNCHER, 1, "AT DIS               ");
        }
    }

}
