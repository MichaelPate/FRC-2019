/******************************************************************************|
|                    __      ____        ___       ____                        |
|                   /_ |    |___ \      / _ \     |___ \                       |
|                    | |      __) |    | | | |      __) |                      |
|                    | |     |__ <     | | | |     |__ <                       |
|                    | |     ___) |    | |_| |     ___) |                      |
|                    |_|    |____/      \___/     |____/                       |
|                                                                              |
|******************************************************************************|
|                                                                              |
|     File:     Robot.java                                                     |
|     Type:     Tele-op                                                        |
|     Version:  100 (1.0.0)                                                    |
|                                                                              |
|     Author:   Michael Pate                                                   |
|     Date:     January 22, 2019                                               |
|     Team:     1303, WYOHAZARD                                                |
|     Host:     Casper College, ROBO 1616                                      |
|                                                                              |
|     Description:                                                             |
|         This is the first official rendition of the robotic teleop program   |
|         for the 2019/20 FIRST FRC season. In brief, this teleop includes     |
|         control over the chassis and its mechanism along with simple         |
|         diagnostics reporting.                                               |
|                                                                              |
|     Robot Details and Electrical Specifications:                             |
|         Drivetrain:                                                          |
|            -4x4 mecanum wheelbase, standard CIM motor and CIMPLE box drive   |
|            -SPARK motor controllers, using PWM protocol                      |
|         Mechanism:                                                           |
|            -CIM motor w/ custom gearbox assy. for mech. lift                 |
|            -CIM motor w/out gearing for mech. actuation                      |
|            -Multiple small pnuematic cylinders for mech. actuation           |
|            -Multiple large pnuematic cylinders for chassis lift              |
|         Support:                                                             |
|            -Pnuematic system                                                 |
|                *** Includes compressor, tanks, regulation, etc. ***          |
|            -Standard 12 volt sealed lead acid battery                        |
|            -SPARK motor controllers w/ PWM protocol for all motors           |
|            -Standard NI RoboRIO for main control                             |
|            -Inline fuses on all sensetive components                         |
|                *** RoboRIO, WiFi radio, etc. ***                             |
|         Environment / Sensors:                                               |
|            -Dual Logitech USB cameras, 720p @ 30fps                          |
|            -Multiple IR based analog distance sensors                        |
|            -Digital pnuematic air pressure detection                         |
|            -Analog temperature detection                                     |
|                *** motor ctrls., RoboRIO, compressor, cabin, etc. ***        |
|            -Limit switches where appropriate                                 |
|                                                                              |
|     Notes:                                                                   |
|         All softwares, utilities, and firmwares are provided by FIRST and    |
|         are up to the latest version for the 2019/20 season.                 |
|                                                                              |
|         All constants are formatted as TYPE_DESC for clarity.                |
|                                                                              |
|******************************************************************************/

package frc.robot;

import java.text.DecimalFormat;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// The VM on the RIO is configured to run an instance of this class.
public class Robot extends TimedRobot {
    ////////////////////////////////////////////////////////////////////////////
    //      Pre-included variables and user constants below.                  //
    ////////////////////////////////////////////////////////////////////////////
    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "A 100";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    private boolean useDiagnostics = true;

    private final double LIMIT_TELEOPFORWARD = 0.5;
    private final double LIMIT_TELEOPREVERSE = -0.5;
    private final double LIMIT_TESTFORWARD = 0.25;
    private final double LIMIT_TESTREVERSE = -0.25;

    private final double TRIM_CHASSISSTICK_X = 0.0;
    private final double TRIM_CHASSISSTICK_Y = 0.0;
    private final double TRIM_CHASSISSTICK_LT = 0.0;
    private final double TRIM_CHASSISSTICK_RT = 0.0;

    ////////////////////////////////////////////////////////////////////////////
    //      Robot hardware and sensors defined below.                         //
    ////////////////////////////////////////////////////////////////////////////

    private final int PORT_DRIVESTICK = 0;
    private final Joystick chassisStick = new Joystick(PORT_DRIVESTICK);

    private final int PORT_MFL = 0;
    private final int PORT_MFR = 1;
    private final int PORT_MRL = 2;
    private final int PORT_MRR = 3;
    private final PWMVictorSPX chassisMotor_fl = new PWMVictorSPX(PORT_MFL);
    private final PWMVictorSPX chassisMotor_fr = new PWMVictorSPX(PORT_MFR);
    private final PWMVictorSPX chassisMotor_rl = new PWMVictorSPX(PORT_MRL);
    private final PWMVictorSPX chassisMotor_rr = new PWMVictorSPX(PORT_MRR);

    ////////////////////////////////////////////////////////////////////////////
    //      Pre-included methods below.                                       //
    ////////////////////////////////////////////////////////////////////////////

    /*  void robotInit(void)
    *   1. Called once upon robot startup
    *   2. Precondition: Robot was just powered on and is communicating
    *   3. No postcondition defined.
    */
    @Override
    public void robotInit()
    {
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("Auto", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);
    }

    /*  void robotPeriodic(void)
    *   1. Called for every robot packet.
    *   2. Precondition: Robot is communicating with driver station
    *   3. No postcondition defined.
    */
    @Override
    public void robotPeriodic()
    {
        // Read the diagnostic button on the dashboard
        useDiagnostics = SmartDashboard.getBoolean("DB/Button 0", false);
    }

    /*  void autonomousInit(void)
    *   1. Runs once when an autonomous is selected
    *   2. Precondition: Robot is communicating and an auto. was chosen
    *   3. Postcondition: The chosen autonomous's name is relayed to
    *      autonomousPeriodic() and the dashboard.
    */
    @Override
    public void autonomousInit()
    {
        m_autoSelected = m_chooser.getSelected();
        System.out.println("Auto selected: " + m_autoSelected);
    }

    /*  void autonomousPeriodic(void)
    *   1. Ran repeatedly when an autonomous is running
    *   2. Precondition: Robot is communicating and an auto. is running
    *   3. No postcondition defined.
    */
    @Override
    public void autonomousPeriodic()
    {
        switch (m_autoSelected)
        {
            case kCustomAuto:
            // Custom Auto here.
            break;
            case kDefaultAuto:
            default:
            // Default Auto here.
            break;
        }
    }

    /*  void teleopPeriodic(void)
    *   1. Ran repeatedly during teleop, handles chassis drive and mechanism
    *   2. Precondition: Robot is communicating and a teleop is running
    *   3. No postcondition defined.
    */
    @Override
    public void teleopPeriodic()
    {
        // This method handles everything chassis drive based, so nothing else needed.
        chassisDrive(LIMIT_TELEOPFORWARD, LIMIT_TELEOPREVERSE, useDiagnostics);

        //TODO: Finish teleopPeriodic()
    }

    /*  void testPeriodic(void)
    *   1. Ran repeatedly during test
    *   2. Precondition: Robot is communicating and a test is running
    *   3. No postcondition is defined.
    */
    @Override
    public void testPeriodic()
    {
        // Test run the chassis, and force diagnostics
        chassisDrive(LIMIT_TESTFORWARD, LIMIT_TESTREVERSE, true);

        //TODO: for testPeriodic, copy teleopPeriodic but force diagnostics when possible.
    }

    ////////////////////////////////////////////////////////////////////////////
    //      Robot-specific methods below.                                     //
    ////////////////////////////////////////////////////////////////////////////

    /*  void chassisDrive(double forwardLimit, double reverseLimit)
     *  Handles controller input and chassis drive
     *  @param: forwardLimit/reverseLimit define the max speeds for the motors (-1 to 1)
     *          diagnostics, if true will output to terminal and wont if false
     *  @pre  : one USB controllers is connected to DS and robot has comms.
     *  @post : the chassis drive motors are powered accordingly
     */ 
    void chassisDrive(double forwardLimit, double reverseLimit, boolean diagnostics)
    {
        // Step 1: Read the drive stick axes.
        double x = chassisStick.getX();
        double y = -chassisStick.getY();
        double lt = -chassisStick.getRawAxis(2);
        double rt = -chassisStick.getRawAxis(3);

        // Step 2: Apply any trim.
        x += TRIM_CHASSISSTICK_X;
        y += TRIM_CHASSISSTICK_Y;
        lt += TRIM_CHASSISSTICK_LT;
        rt += TRIM_CHASSISSTICK_RT;

        // Step 3: Compute the powers for the motors.
        double powerFl, powerFr, powerRl, powerRr;
        double eqA = ((x + y)/(Math.abs(x + y))) * Math.sqrt((x * x) + (y * y)) * 1.0;
        double eqB = ((y - x)/(Math.abs(y - x))) * Math.sqrt((x * x) + (y * y)) * 1.0;

        if (x + y == 0) {
            eqA = 0;
            eqB = 0;
        }

        powerFl = eqA;// + (rt - lt);
        powerRr = eqA;// + (lt - rt);
        powerFr = eqB;// + (lt - rt);
        powerRl = eqB;// + (rt - lt);
        //TODO: Apply rotation from the triggers for the chassisDrive() method.

        // Step 4: Map the computed values to be within the power limits.
        powerFl = map(powerFl, -1.2, 1.2, reverseLimit, forwardLimit);
        powerFr = map(powerFr, -1.2, 1.2, reverseLimit, forwardLimit);
        powerRl = map(powerRl, -1.2, 1.2, reverseLimit, forwardLimit);
        powerRr = map(powerRr, -1.2, 1.2, reverseLimit, forwardLimit);

        // Step 5: Apply the powers to the motors, remember that left side motors are reversed.
        chassisMotor_fl.setSpeed(-powerFl);
        chassisMotor_fr.setSpeed(powerFr);
        chassisMotor_rl.setSpeed(-powerRl);
        chassisMotor_rr.setSpeed(powerRr);

        // Step 6: If in diagnostics, output information to the terminal.
        if (diagnostics)
        {
            // Its best to round the values for display.
            DecimalFormat df = new DecimalFormat("#.##");

            SmartDashboard.putString("DB/String 0", "Stick X: " + df.format(x));
            SmartDashboard.putString("DB/String 1", "Stick Y: " + df.format(y));
            SmartDashboard.putString("DB/String 3", "FL: " + df.format(-powerFl) + " --- FR: " + df.format(powerFr));
            SmartDashboard.putString("DB/String 4", "RL: " + df.format(-powerRl) + " --- RR: " + df.format(powerRr));
        }

        return;
    }

    // Helper function map()
    // Maps a range of values to another.
    //TODO: Fix all method headers to be standard.
    //private double map(double in, double inMin, double inMax, double outMin, double outMax)
    //{
    //    return (in - inMin) * (outMax - outMin) / (inMax - inMin) * outMin;
    //}

    private double map(double mapVal, double inMin, double inMax, double outMin, double outMax) {
        return (mapVal - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

}