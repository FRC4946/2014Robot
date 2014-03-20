/*
 * Autonumous routine:
 *    1. start 90 degrees
 *    2. get second ball and face the goal
 *    3. drive forward to distance
 *    4. Shoot ball 1 hot(has failsafe)s
 *    5. Intake ball 2 and shoot hot(has failsafe)
 */
package org.usfirst.frc4946.autoMode;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.CriteriaCollection;
import edu.wpi.first.wpilibj.image.NIVision;
import edu.wpi.first.wpilibj.image.NIVisionException;
import org.usfirst.frc4946.DistanceSensor;
import org.usfirst.frc4946.IntakeArm;
import org.usfirst.frc4946.Launcher;
import org.usfirst.frc4946.Loader;
import org.usfirst.frc4946.RobotConstants;
import org.usfirst.frc4946.vision.TargetReport;
import org.usfirst.frc4946.vision.Vision;
import org.usfirst.frc4946.vision.VisionConstants;

/**
 *
 * @author Stefan
 */
public class AutoTwoBallWithTurning extends AutoMode {

    //AutoMode autoRoutine = new AutoMode(m_robotDrive, m_launcher, m_loader, m_intakeArm, m_distanceSensor);
    int step = -1;
    int counter = 0;
    int atDistanceCount = 0;
    double hotGoalOneTime = 0.0;
    double shootTime = 0.0;
    TargetReport m_target = new TargetReport();

    Vision m_vision = new Vision();

    AxisCamera camera;          // the axis camera object (connected to the switch)
    CriteriaCollection cc;      // the criteria for doing the particle filter operation

    Timer m_timer = new Timer();

    public AutoTwoBallWithTurning(RobotDrive drive, Launcher launcher, Loader loader, IntakeArm intakeArm, DistanceSensor distanceSensor) {
        super(drive, launcher, loader, intakeArm, distanceSensor);
    }

    public void init() {

        m_timer.reset();
        m_timer.start();

        camera = AxisCamera.getInstance();  // get an instance of the camera
        cc = new CriteriaCollection();      // create the criteria for the particle filter
        cc.addCriteria(NIVision.MeasurementType.IMAQ_MT_AREA, VisionConstants.AREA_MINIMUM, 65535, false);

        initGyroSensor();
        step = -1;
        counter = 0;
        atDistanceCount = 0;
        hotGoalOneTime = 0.0;

        extendArm();

        startShooter(true);
        setShooterSpeed(1425, true);
        m_driverStation.println(RobotConstants.AUTO_LCD_LAUNCHER, 1, "SHOOTER ON 1425               ");
    }

    public void run() {
        m_launcher.update();
        counter++;
        m_driverStation.println(RobotConstants.AUTO_LCD_INTAKE, 1, "Dist " + m_distanceSensor.getRangeInchs() + "                          ");
        if (step == -2) {
            enableRollers();
        }
        if (m_timer.get() > 0.1) {
            step = -1;
        }
        if (step==-1){
            if (turnToAngle(1,45)){
                step=0;
            }
        }
        if (step == 0) {
            driveToDistance(9 * 12, 0.4);
        }

        if (atDistance(9 * 12) && step == 0) {
            atDistanceCount++;
            if (atDistanceCount > 3) {
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

            //vision
            BinaryImage image = m_vision.getFilteredImage(camera, cc);
            if (m_vision.areParticles(image)) {
                m_vision.detectTargets(image);
                m_target = m_vision.getBestTarget(image);
                hotGoalOneTime = m_timer.get();
            }
            try {
                image.free();
            } catch (NIVisionException ex) {
                ex.printStackTrace();
            }
            // end vision
        }
            //shoot after ball has settled 
        //check the time we are shooting at and know if it's hot or not
        if (step == 1 && counter > 100) {
            hotGoalOneTime = m_timer.get();
            if (m_target.Hot) {
                extendLoader();
                shootTime = m_timer.get();
                counter = 0;
            }
            if (m_timer.get()>5) {
                extendLoader();
                shootTime = m_timer.get();
                counter = 0;
            }
            if (shootTime <= 5.0) {
                m_driverStation.println(RobotConstants.AUTO_LCD_LOADER, 1, "SHOOTING1HOT                       ");
                shootTime =+ 0.15;
            } else {
                m_driverStation.println(RobotConstants.AUTO_LCD_LOADER, 1, "SHOOTING1COLD                       ");
                shootTime =+ 0.15;
            }
        }
        //reset our loader after we are sure we have fired
        if (step == 1 && m_timer.get() > shootTime && shooterIsAtTargetSpeed(1425)) {
            retractArm();
            step = 2;       //next ball
            counter = 0;
        }
        
        if (step == 2 && counter > 150) {
            //toggle intake on and leave on just in case ... maybe turn off but I would prefer not to
            enableRollers();
            //if limitswitchs check that ball is in robot.
            counter = 0;
            step = 3;
        }
        if (step == 3 && counter > 100) {
           
            //vision for second goal as double check (only takes little while to process
            BinaryImage image = m_vision.getFilteredImage(camera, cc);
            if (m_vision.areParticles(image)) {
                m_vision.detectTargets(image);
                m_target = m_vision.getBestTarget(image);
            }
            try {
                image.free();
            } catch (NIVisionException ex) {
                ex.printStackTrace();
            }
            /// vision end
            if (m_target.Hot) {
                extendLoader();
                shootTime = m_timer.get();
                m_driverStation.println(RobotConstants.AUTO_LCD_LOADER, 1, "SHOOTING2HOT                       ");
            }
                         
            //failsafe
             if (shootTime > 8.5) {
                extendLoader();
                m_driverStation.println(RobotConstants.AUTO_LCD_LOADER, 1, "SHOOTING2                       ");
            }
        }
        
        
        
    }
}
