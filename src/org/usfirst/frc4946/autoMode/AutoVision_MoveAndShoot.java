/*
 * Autonumous routine:
 *    1. Move forwards, until 10 feet away
 *    2. Shoot the ball
 */

package org.usfirst.frc4946.autoMode;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.CriteriaCollection;
import edu.wpi.first.wpilibj.image.NIVision;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;
import org.usfirst.frc4946.DistanceSensor;
import org.usfirst.frc4946.IntakeArm;
import org.usfirst.frc4946.Launcher;
import org.usfirst.frc4946.Loader;
import org.usfirst.frc4946.RobotConstants;

import org.usfirst.frc4946.vision.*;

/**
 *
 * @author Stefan
 */
public class AutoVision_MoveAndShoot extends AutoMode {

    //AutoMode autoRoutine = new AutoMode(m_robotDrive, m_launcher, m_loader, m_intakeArm, m_distanceSensor);
    int step = 0;
    int counter = 0;
    int atDistanceCount = 0;
    TargetReport m_target = new TargetReport();
    
    Vision m_vision;
    
    ///AxisCamera camera;          // the axis camera object (connected to the switch)
    CriteriaCollection cc;      // the criteria for doing the particle filter operation

    Timer m_timer = new Timer();
    
    public AutoVision_MoveAndShoot(RobotDrive drive, Launcher launcher, Loader loader, IntakeArm intakeArm, DistanceSensor distanceSensor) {
        super(drive, launcher, loader, intakeArm, distanceSensor);
    }

    public void init() {
    
        m_timer.reset();
        m_timer.start();
        
        //camera = AxisCamera.getInstance();  // get an instance of the camera
        cc = new CriteriaCollection();      // create the criteria for the particle filter
        cc.addCriteria(NIVision.MeasurementType.IMAQ_MT_AREA, VisionConstants.AREA_MINIMUM, 65535, false);
                
        //initGyroSensor();
        
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
        //m_driverStation.println(RobotConstants.AUTO_LCD_INTAKE, 1, "Dist " + m_distanceSensor.getRangeInchs()+"                          ");
        if (step == 0) {
            //driveToDistance(9 * 12, 0.8);

        

        //if (atDistance(9 * 12) && step == 0) {
        //    atDistanceCount++;
        //    if (atDistanceCount > 2) {
        //        drive(0.0, 0.0);
        //    }
        //} else {
        //    atDistanceCount = 0;
        //
        //}

        //if (atDistanceCount > 10 && step == 0) {
            step = 1;
        //    drive(0.0, 0.0);
        //    counter = 0;
        //    m_driverStation.println(RobotConstants.AUTO_LCD_LAUNCHER, 1, "AT DIS & SPEED               ");
        //    s
            BinaryImage image = m_vision.getFilteredImage(cc);
            if (m_vision.areParticles(image)) {
                
                m_vision.detectTargets(image);
                m_target = new TargetReport(); // Reset all the values to default
                m_target = m_vision.getBestTarget(image);
                
                m_driverStation.println(RobotConstants.AUTO_LCD_LAUNCHER,1,"Tape: "+m_target.tapeWidthScore);
                m_driverStation.println(RobotConstants.AUTO_LCD_INTAKE,1,"Height: "+m_target.verticalScore);                
                
                m_driverStation.updateLCD();
                // If the target is more than ~6.5 feet to the side, assume it is on the other side of the goal and flip our HOT flag.
                
                
                /******************************************************* NEEDS TESTING!!!!! *******************************************************/
                //double distance = m_vision.getDistance(image, m_target);
                //if(distance>11){
                //    m_target.Hot = !m_target.Hot;
                //}
            }
            try {
                image.free();
            } catch (NIVisionException ex) {
                ex.printStackTrace();
            }
            
        }

        if (step == 1 && counter > 100) {
            
            //m_driverStation.println(RobotConstants.AUTO_LCD_LAUNCHER, 1, "IsHot: "+m_target.Hot+"                      ");
            
            // If the target is hot, GO GO GO!!!
            if (m_target.Hot) {
                extendLoader();
                m_driverStation.println(RobotConstants.AUTO_LCD_LOADER, 1, "SHOOTING                       ");
            }
            
            // If the target was not hot, wait 5 seconds for the targets to switch and then GO GO GO!!!
            else if (!m_target.Hot && m_timer.get() > 5){
                extendLoader();
                m_driverStation.println(RobotConstants.AUTO_LCD_LOADER, 1, "SHOOTING                       ");
            }
            
            // Otherwise, DON'T GO YET!!!
            else{
                m_driverStation.println(RobotConstants.AUTO_LCD_LOADER, 1, "WAITING TO SHOOT...                       ");
            }

        }
    }

}
