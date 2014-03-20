package org.usfirst.frc4946;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

public class IntakeArm {

    private final Talon m_intakeController = new Talon(RobotConstants.PWM_MOTOR_INTAKE);

    private final Solenoid m_extendGrabberSolenoid = new Solenoid(RobotConstants.EXTEND_ARM_VALVE_RELAY);
    private final Solenoid m_retractGrabberSolenoid = new Solenoid(RobotConstants.RETRACT_ARM_VALVE_RELAY);

    DriverStationLCD m_driverStationLCD = DriverStationLCD.getInstance();
    
    //private double speed = 0.0;
    private boolean old_MotorsAreEnabled = false;
    private boolean motorsAreEnabled = false;
    private boolean armIsExtended = false;
    private boolean rollersAreReversed = false;
    
    private int m_solenoidCounter = 0;

    private final DriverStation m_driverStation;

    
    
    
    IntakeArm() {
        LiveWindow.addActuator("Intake Arm", "Roller", m_intakeController);
        LiveWindow.addActuator("Intake Arm", "Extend solenoid", m_extendGrabberSolenoid);
        LiveWindow.addActuator("Intake Arm", "Retract solenoid", m_retractGrabberSolenoid);

        m_driverStation = DriverStation.getInstance();
    }

    
    
    public void toggleExtended() {
        armIsExtended = !armIsExtended;

        setExtended(armIsExtended);
    }

    /**
     * Extend or retract the arm.
     *
     * @param extend Whether to extend or retract the arm.
     */
    public void setExtended(boolean extend) {

        armIsExtended = extend;

        if (extend) {
            m_retractGrabberSolenoid.set(false);
            m_extendGrabberSolenoid.set(true);
            m_solenoidCounter = RobotConstants.SOLENOID_COOLDOWN_TIME;

        } else {

            //And retract the arm
            m_extendGrabberSolenoid.set(false);
            m_retractGrabberSolenoid.set(true);
            m_solenoidCounter = RobotConstants.SOLENOID_COOLDOWN_TIME;
        }
    }

    public void updateSolenoids() {
        // Turn off the solenoid after 100 cycles.
        if (m_solenoidCounter == 0) {
            m_extendGrabberSolenoid.set(false);
            m_retractGrabberSolenoid.set(false);

        } else {
            m_solenoidCounter--;

        }
    }

    public boolean getExtendedState() {
        return armIsExtended;
    }

    
    
    public void toggleEnabled() {
        motorsAreEnabled = !motorsAreEnabled;
        old_MotorsAreEnabled = motorsAreEnabled;

        rollersAreReversed = false;
        setEnabledRollers(motorsAreEnabled);
    }

    /**
     * Start or stop the motors. If the speed has been changed, call
     * setEnabled(true) to update the motors.
     *
     * @param isEnabled Whether to enable the motors or not.
     */
    public void setEnabledRollers(boolean isEnabled) {

        motorsAreEnabled = isEnabled;

        if (isEnabled && !rollersAreReversed) {
            m_intakeController.set(1.0);
        }else if (isEnabled && rollersAreReversed) {
            m_intakeController.set(-1.0);
        } else {
            m_intakeController.set(0.0);
        }

    }

    public void reverseRollers(boolean isReversed){
        rollersAreReversed = isReversed;
        if (isReversed) {
            old_MotorsAreEnabled = motorsAreEnabled;
            setEnabledRollers(true);
        } else {
            motorsAreEnabled = old_MotorsAreEnabled;
            setEnabledRollers(motorsAreEnabled);
        }


    }


//    /**
//     * Set the speed of the motors without checking the result. After calling
//     * this, call setEnabled(true) to update the motors.
//     *
//     * @param power The speed to set. Should be between -12 and 12 ish
//     */
//    public void setSpeedOpenLoop(double power) {
//
//        speed = power / m_driverStation.getBatteryVoltage();
//        
//        // Old method, using just a percentage
//        //if(power >= -1.0 && power <= 1.0){
//        //  speed = power;
//        //}
//    }

}
