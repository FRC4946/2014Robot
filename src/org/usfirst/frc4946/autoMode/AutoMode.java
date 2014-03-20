/*
 * This is the master autonomous class
 * Every other auto class inherits from this
 */
package org.usfirst.frc4946.autoMode;

import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.PIDController;
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
public abstract class AutoMode {

    RobotDrive m_robotDrive;
    Launcher m_launcher;
    Loader m_loader;
    IntakeArm m_intakeArm;
    DistanceSensor m_distanceSensor;
    protected DriverStationLCD m_driverStation = DriverStationLCD.getInstance();
    Gyro m_gyro = new Gyro(RobotConstants.GYRO_SENSOR);

    AutoMode(RobotDrive drive, Launcher launcher, Loader loader, IntakeArm intakeArm, DistanceSensor distanceSensor) {
        m_robotDrive = drive;
        m_launcher = launcher;
        m_loader = loader;
        m_intakeArm = intakeArm;
        m_distanceSensor = distanceSensor;
    }

    public void initGyroSensor() {
       m_gyro.reset();
    }

    protected boolean shooterIsAtTargetSpeed(double speed) {
        return getCurrentShooterSpeed() >= (speed - 50) && getCurrentShooterSpeed() <= (speed + 50);

    }

    public void driveToDistance(double distance, double speed) {
        double currentDistance = m_distanceSensor.getRangeInchs();

        if (currentDistance >= distance && RobotConstants.DISTANCE_SENSOR_RANGE <= Math.abs(currentDistance - distance)) {
            //double angle = m_gyro.getAngle();
            //double correctedAngle = angle*-0.03;

            drive(speed, 0);
            //drive(speed, correctedAngle);
        }
        //if (currentDistance <= distance && RobotConstants.DISTANCE_SENSOR_RANGE <= Math.abs(currentDistance - distance)) {
        //  drive(-speed, 0);
        //}

    }

    public void drive(double speed, double turn) {
        m_robotDrive.drive(speed, turn);
    }

    public boolean atDistance(double distance) {
        double currentDistance = m_distanceSensor.getRangeInchs();
        return RobotConstants.DISTANCE_SENSOR_RANGE >= Math.abs(currentDistance - distance);

    }

    public void driveDistance(double distance, double speed) {
        //get current distance
        //subtract input distance from current distance and drive to that distance
        double currentDistance = m_distanceSensor.getRangeInchs();
        driveToDistance(currentDistance - distance, speed);

    }

    public boolean turnToAngle(double speed,int angle) {
        //needs work, potentially use the gyro,compass, combo part we have?
        
        // basic p controller for turning to the correct angle and not overshooting  
        int threshold=2; //2 degree leanance 
        
        if(threshold>Math.abs(angle-m_gyro.getAngle())){
            return true;
        }
        
        double p = (Math.abs(angle-m_gyro.getAngle()) * speed / 180);
        if (angle>m_gyro.getAngle()+threshold){
            drive(0,-p);          // Linear turning speed
            //drive(0,-speed*p);    // Curved turning speed
        }
        else if (angle<m_gyro.getAngle()-threshold){
            drive(0,p);          // Linear turning speed
            //drive(0,speed*p);    // Curved turning speed
        }
        
        return false;
    }

    public void updateShooter(boolean closedLoop) {
        m_launcher.update();
    }

    public void startShooter(boolean closedLoop) {
        if (closedLoop) {
            m_launcher.setClosedLoopEnabled(true);
        } else {
            m_launcher.setOpenLoopEnabled(true);
        }

    }

    public void setShooterSpeed(double speed, boolean closedLoop) {
        if (closedLoop == true) {
            m_launcher.setSpeedRPM(speed);

        } else {
            m_launcher.setSpeedOpenLoop(speed);

        }
    }

    public void stopShooter(boolean closedLoop) {
        if (closedLoop) {
            m_launcher.setClosedLoopEnabled(false);
        } else {
            m_launcher.setOpenLoopEnabled(false);
        }

    }

    public void extendArm() {
        m_intakeArm.setExtended(true);
        m_intakeArm.updateSolenoids();
    }

    public void retractArm() {
        m_intakeArm.setExtended(false);
        m_intakeArm.updateSolenoids();
    }

    public void enableRollers() {
        m_intakeArm.setEnabledRollers(true);
    }

    public void disableRollers() {
        m_intakeArm.setEnabledRollers(false);
    }

    public void extendLoader() {
        m_loader.setExtended(true);
        m_loader.updateSolenoids();
    }

    public void retractLoader() {
        m_loader.setExtended(false);
        m_loader.updateSolenoids();
    }

    public double getCurrentShooterSpeed() {
        //return the speed
        double rpm = m_launcher.getSpeedRPM();
        return rpm;
    }
}
