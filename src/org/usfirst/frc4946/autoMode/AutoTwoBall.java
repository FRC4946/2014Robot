/*
 * Autonumous routine:
 *    1. Drive to distance (drop intake to drag ball)
 *    2. Check hot(if hot shoot)
 *    3. Intake second ball
 *    4. Move forwards, until 8 feet away
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
public class AutoTwoBall extends AutoMode {

    //AutoMode autoRoutine = new AutoMode(m_robotDrive, m_launcher, m_loader, m_intakeArm, m_distanceSensor);
    int step = -1;
    int counter = 0;
    int atDistanceCount = 0;
    double hotGoalOneTime = 0.0;
    boolean didShoot = false;
    TargetReport m_target = new TargetReport();

    Vision m_vision = new Vision();

    AxisCamera camera;          // the axis camera object (connected to the switch)
    CriteriaCollection cc;      // the criteria for doing the particle filter operation

    Timer m_timer = new Timer();

    public AutoTwoBall(RobotDrive drive, Launcher launcher, Loader loader, IntakeArm intakeArm, DistanceSensor distanceSensor) {
        super(drive, launcher, loader, intakeArm, distanceSensor);
    }

    public void init() {

        m_timer.reset();
        m_timer.start();

        camera = AxisCamera.getInstance();  // get an instance of the camera
        cc = new CriteriaCollection();      // create the criteria for the particle filter
        cc.addCriteria(NIVision.MeasurementType.IMAQ_MT_AREA, VisionConstants.AREA_MINIMUM, 65535, false);

        //initGyroSensor();
        step = -1;
        counter = 0;
        atDistanceCount = 0;
        hotGoalOneTime = 0.0;
        didShoot = false;

        extendArm();

        startShooter(true);
        setShooterSpeed(1425, true);
        m_driverStation.println(RobotConstants.AUTO_LCD_LAUNCHER, 1, "SHOOTER ON 1425               ");
    }

    public void run() {
        m_launcher.update();
        counter++;
        m_driverStation.println(RobotConstants.AUTO_LCD_INTAKE, 1, "Dist " + m_distanceSensor.getRangeInchs() + "                          ");
        if (step == -1) {
            enableRollers();
        }
        if (m_timer.get() > 0.1) {
            step = 0;
        }
        if (step == 0) {
            driveToDistance(9 * 12, 0.75);
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
                m_driverStation.println(RobotConstants.AUTO_LCD_LOADER, 1, "SHOOTING1HOT                       ");
                counter = 0;
                didShoot = true;
            }
            else if (m_timer.get()>5) {
                extendLoader();
                m_driverStation.println(RobotConstants.AUTO_LCD_LOADER, 1, "SHOOTING1COLD                       ");
                counter = 0;
                didShoot = true;
            }
        }
        //reset our loader after we are sure we have fired
        if (step == 1 && counter > 100 && didShoot == true) {
            retractLoader();
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
                m_driverStation.println(RobotConstants.AUTO_LCD_LOADER, 1, "SHOOTING2HOT                       ");
                step = 4;
            }
                         
            //failsafe
             if (m_timer.get() > 8.5) {
                extendLoader();
                m_driverStation.println(RobotConstants.AUTO_LCD_LOADER, 1, "SHOOTING2                       ");
                step = 4;
             }
        }
        
        
        
    }
}
