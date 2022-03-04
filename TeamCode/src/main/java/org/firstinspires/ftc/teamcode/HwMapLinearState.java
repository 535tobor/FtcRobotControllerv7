package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;

/**
 * Class that contains the hardware map and code operations for linear use
 */
public class HwMapLinearState extends LinearOpMode {

    public DcMotor fl, fr, br, bl, carouselSpinner, intakeL, intakeR, extender;
    public DistanceSensor dsL, dsR, dsBR;
    public Servo basket;
    public CRServo turnExtender;
    public TouchSensor tsL, tsR;

    public int rotations = 0;
    public BNO055IMU imu;
    public Orientation gyroAngles;
    public double desiredRobotHeading;
    public final double WHEEL_DIAMETER = 5.65, COUNTS_PER_INCH = 1120/(WHEEL_DIAMETER * 3.14);
    public ElapsedTime runtime = new ElapsedTime();
    public String side = "empty";

    /**
     * Sets up hardware map for chassis and other mechanisms
     */
    public void initHwMap()
    {
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        carouselSpinner = hardwareMap.dcMotor.get("carousel");
        dsL = hardwareMap.get(DistanceSensor.class, "ds"); //left
        dsR = hardwareMap.get(DistanceSensor.class, "ds2"); //right
        dsBR = hardwareMap.get(DistanceSensor.class, "dsBR");
        basket = hardwareMap.servo.get("basket");
        turnExtender = hardwareMap.get(CRServo.class,"turnEx");
        extender = hardwareMap.dcMotor.get("extender");
        tsL = hardwareMap.touchSensor.get("tsL");
        tsR = hardwareMap.touchSensor.get("tsR");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    @Override
    public void runOpMode()  {}


    /**
     * Powers the carousel spinning motor for a variable amount of time
     * @param time in seconds
     */
    public void spin(double time)
    {
        runtime.reset();
        while (opModeIsActive()&& runtime.time() < time){
            carouselSpinner.setPower(.42);
        }
        carouselSpinner.setPower(0);
    }

    /**
     * Updates current robot angle according to IMU Gyro, use in a loop
     */
    public void updateGyro()
    {
        gyroAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    /**
     * @return the angle of the robot according to the IMU Gyro
     */
    public double getRobotAngle() {
        if (desiredRobotHeading - (rotations * 360 + gyroAngles.firstAngle) > 200) {
            rotations++;
        } else if (desiredRobotHeading - (rotations * 360 + gyroAngles.firstAngle) < -200) {
            rotations--;
        }
        return (rotations * 360 + gyroAngles.firstAngle);
    }

    /**
     * Set a run mode to the chassis motors
     * @param mode motor run mode
     */
    public void setModeAll(DcMotor.RunMode mode)
    {
        fl.setMode(mode); fr.setMode(mode); bl.setMode(mode); br.setMode(mode);
    }

    /**
     * Set a power to motors on the left side of the chassis
     * @param power from -1.0 to 1.0
     */
    public void setPowerLeft(double power)
    {
        fl.setPower(power); bl.setPower(power);
    }

    /**
     * Set a power to motors on the right side of the chassis
     * @param power from -1.0 to 1.0
     */
    public void setPowerRight(double power)
    {
        fr.setPower(power); br.setPower(power);
    }

    /**
     * Set a power to all motors of the chassis
     * @param power from -1.0 to 1
     */
    public void setPowerAll(double power)
    {
        setPowerLeft(power); setPowerRight(power);
    }

    /**
     * Stop all chassis motors by setting them to zero
     */
    public void setPowerZero()
    {
        setPowerAll(0);
    }

    /**
     * Drive forward autonomously based on time
     * @param speed from -1.0 to 1.0
     * @param time in seconds
     */
    public void driveByTime(double speed, double time){
        runtime.reset();
        while (opModeIsActive() && runtime.time() < time ){
            setPowerAll(speed);
        }
        setPowerZero();
    }

    /**
     * Drive and change the orientation of the robot based on the IMU gyro
     * @param desiredDegrees from current angle
     * @param timeoutS in seconds and used to force stop the method; timeout
     */
    public void robotTurn(double desiredDegrees, double timeoutS) //from current
    {

        double initAngle = getRobotAngle();
        double desiredAngle = initAngle + desiredDegrees;
        runtime.reset();
        while(opModeIsActive()&&(runtime.time()<timeoutS)&&(threshold(getRobotAngle(), desiredAngle, 2)))
        {//left is pos
            updateGyro();
            if(desiredDegrees>0)
            {
                setPowerLeft(-.7);
                setPowerRight(.7);
            }
            else
            {
                setPowerLeft(.7);
                setPowerRight(-.7);
            }
        }
        setPowerZero();
    }

    public void robotTurnToAngle(int degrees, double timeoutS)
    {
        double power = .06;
        int multiplierL = 1, multiplierR = -1;
        double divisor = degrees - getRobotAngle(); //desired - initial
        if(getRobotAngle()>degrees)
        {
            multiplierL = -1;
            multiplierR = 1;
        }
        runtime.reset();
        while(opModeIsActive()&&threshold(getRobotAngle(), degrees, 2) && Math.abs(power)>.055&&(runtime.time()<timeoutS));
        {
            updateGyro();
            power = (getRobotAngle()-degrees)/divisor; // (current - desired) / desired - initial
            setPowerLeft(power*multiplierL);
            setPowerRight(power*multiplierR);
        }
        setPowerZero();

    }

    /**
     * Drives the chassis autonomously using the motors' built in encoders with an algorithm that averages the counts traveled instead of Run to position
     * @param Lspeed from -1.0 to 1.0
     * @param Rspeed from -1.0 to 1.0
     * @param Inches the robot moves this increment
     * @param timeoutS in seconds and used to force stop the method; timeout
     * @param rampup increment used to "Slowly" ramp the motors up over time
     */
    public void encoderDrive(double Lspeed, double Rspeed, double Inches, double timeoutS, double rampup){
        int newLeftTarget = (fl.getCurrentPosition() + bl.getCurrentPosition() )/2 + (int)(Inches * COUNTS_PER_INCH);
        int newRightTarget= (fr.getCurrentPosition() + br.getCurrentPosition() )/2 + (int)(Inches * COUNTS_PER_INCH);
        // Ensure that the opmode is still active
        // Determine new target position, and pass to motor controller we only do this in case the encoders are not totally zero'd
        // reset the timeout time and start motion.
        runtime.reset();
        // keep looping while we are still active, and there is time left, and neither set of motors have reached the target
        while ( opModeIsActive() && (runtime.seconds() < timeoutS) && (Math.abs(fl.getCurrentPosition() + bl.getCurrentPosition()) /2
                < newLeftTarget  &&Math.abs(fr.getCurrentPosition() + br.getCurrentPosition())/2 < newRightTarget))
        {
            double rem = (Math.abs(fl.getCurrentPosition())+ Math.abs(bl.getCurrentPosition())+Math.abs(fr.getCurrentPosition()) + Math.abs(br.getCurrentPosition()))/4;
            double NLspeed, NRspeed;
            boolean tele = (Math.abs(fl.getCurrentPosition() + bl.getCurrentPosition()) /2 < newLeftTarget  && Math.abs(fr.getCurrentPosition() + br.getCurrentPosition())/2 < newRightTarget);
            telemetry.update();
            //To Avoid spinning the wheels, this will "Slowly" ramp the motors up over
            //the amount of time you set for this SubRun
            double R = runtime.seconds();
            if (R < rampup) {
                double ramp = R / rampup;
                NLspeed = Lspeed * ramp;
                NRspeed = Rspeed * ramp;
            }
//Keep running until you are about two rotations out
            else if(rem > (2000) )
            {
                NLspeed = Lspeed;
                NRspeed = Rspeed;
            }
            //start slowing down as you get close to the target
            else if(rem > (400) && (Lspeed*.2) > .1 && (Rspeed*.2) > .1) {
                NLspeed = Lspeed * (rem / 2000);
                NRspeed = Rspeed * (rem / 2000);
            }
            //minimum speed
            else {
                NLspeed = Lspeed * .2;
                NRspeed = Rspeed * .2;

            }
            //Pass the seed values to the motors
            setPowerLeft(NLspeed);
            setPowerRight(NRspeed);
        }
        // Stop all motion;
        setPowerZero();
    }

    /**
     * (Integers) Sets up a range/threshold of values between ( b + number < a < b - number ) to account for hardware that cannot reach an exact value but can be stopped within a range
     * @param val value that is being compared (a)
     * @param otherval secondary comparison variable (b)
     * @param range increment/decrement for the range (number)
     * @return the boolean of the threshold
     */
    public boolean threshold(int val, int otherval, int range)
    {
        return (val>( otherval + range)||val<( otherval - range));
    }

    /**
     * (Doubles) Sets up a range/threshold of values between ( b + number < a < b - number ) to account for hardware that cannot reach an exact value but can be stopped within a range
     * @param val value that is being compared (a)
     * @param otherval secondary comparison variable (b)
     * @param range increment/decrement for the range (number)
     * @return the boolean of the threshold
     */
    public boolean threshold(double val, double otherval, double range)
    {
        return (val>( otherval + range)||val<( otherval - range));
    }

    /**
     * Use encoder to power motor without Run To Position
     * @param motor from any on the hardware map
     * @param desiredCounts position in encoder counts
     * @param thresholdRange increment&decrement for range of counts
     * @param power from -1.0 to 1.0
     */
    public void runEncoder(DcMotor motor, int desiredCounts,  int thresholdRange, double power)
    {
        desiredCounts = motor.getCurrentPosition()+desiredCounts;
        power = Math.abs(power);
        if(motor.getCurrentPosition()>desiredCounts)
        {
            power = -power;
        }
        while(opModeIsActive()&& threshold(motor.getCurrentPosition(), desiredCounts, thresholdRange))
        {
            motor.setPower(power);
        }
        motor.setPower(0);
    }

    /**
     * The process of using the motor object's run-to-position call and then idle so the motor is the only hardware running
     * @param motor from any in the hardware map
     * @param counts position in encoder counts
     * @param power from -1.0 to 1.0
     */
    public void motorRTPIdle(DcMotor motor, int counts, double power)
    {

        motorRTP(motor, counts, power);
        while (opModeIsActive() && motor.isBusy())
        {
            idle();
        }
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    /**
     * Intiates the process of using the motor object's run-to-position call; will run in the backgound
     * @param motor from any in the hardware map
     * @param counts position in encoder counts
     * @param power from -1.0 to 1.0
     */
    public void motorRTP(DcMotor motor, int counts, double power)
    {
        motor.setTargetPosition(counts);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    /**
     * Steps to deliver preloaded cargo after intialization in Autonomous
     */
    public void deliverBox()
    {
        int extendCts = 8230, multiplier = 1;
        double boxDeliverDistance = 0;

        runtime.reset();
        while (opModeIsActive() && runtime.time() < 1 && !tsL.isPressed() && !tsR.isPressed())
        { //turn the extender out of the way
            turnExtender.setPower(1);
        }

        motorRTPIdle(extender, extendCts, 1); // extender to tower
        basket.setPosition(.7); //open basket to release piece
        waitFor(.75);
        basket.setPosition(.4); //close basket may be redundant
        motorRTP(extender,-extendCts, -1);// start extender back
        encoderDrive(.7, .7, 8, 5, 1); // drive forward for any turning being done later
    }

    public void deliverBoxParkWH()
    {//drive to the warehouse and near the exit into the shipping hub
        double degreesToFaceWH = 90;
        int multiplier = 1;
        deliverBox(); //deliver box while robot is angled towards tower and stationary, then drive a few inches forward
        if(side.equals("right"))
        {
            multiplier = -1;
        }
        robotTurn(degreesToFaceWH*multiplier, 4); //turn to face the warehouse back wall
        encoderDrive(.9,.9, 28, 5, 1); // drive towards the wall
        robotTurn(-degreesToFaceWH*multiplier, 4);// turn to face the shipping hub

    }

    public void deliverBoxCarousel()
    {// move and turn to park in the depot
        double degreesToFaceWH = 90;
        int multiplier = 1;
        deliverBox(); //deliver box while robot is angled towards tower and stationary, then drive a few inches forward
        if(side.equals("right"))
        {
            multiplier = -1;
        }
        robotTurn(degreesToFaceWH*multiplier, 4); //turn so back of robot is facing the carousel
        //back into carousel using distance sensor and maybe encoders too

        spin(2); // spin the carousel
        encoderDrive(.7,.7,7, 3,1);// drive a little bit forward so robot can turn

        robotTurn(-degreesToFaceWH*multiplier, 4);// turn towards the depot
        encoderDrive(.8,.8,20,5,1);//drive to park in depot




    }

    /**
     * "Interrupt" program for [time] amount of seconds
     * @param time in seconds
     */
    public void waitFor(double time)
    {
        runtime.reset();
        while(opModeIsActive()&& runtime.time()<time)
        {
            telemetry.update();
        }
    }

    public double backDistanceArrayAverage()
    {
        int length = 20;
        double average = 0;
        ArrayList<Double> dsBRValues = new ArrayList<Double>();
        for(int i = 0; i<=length;i++)
        {
            dsBRValues.add(dsBR.getDistance(DistanceUnit.CM));
        }
        for(int j = 0; j<=length;j++)
        {
            average += dsBRValues.get(j);
        }
        average /=length;
        return average;
    }

    /**
     * Drives the chassis autonomously using a distance sensor that calculates the current power proportional to the distance away from an object; goes from fast to slow as it approaches object.
     * @param desiredDistanceFromObj in CM
     * @param multiplier from 0.1 to 1.0; scales down power for motors so they never runs at full speed
     */
    public void driveByDistanceBack(double desiredDistanceFromObj, double multiplier, int timeout)
    {
        double divisor = desiredDistanceFromObj - dsBR.getDistance(DistanceUnit.CM); //desired - initial
        double power = .06;//start power greater than .055 so it complies with the while loop
        multiplier = Math.max(0.1, multiplier);
        multiplier = Math.min(1, multiplier);
        runtime.reset();
        while(opModeIsActive()&&threshold(dsBR.getDistance((DistanceUnit.CM)), desiredDistanceFromObj, 1) && Math.abs(power)>.055&&runtime.time()<timeout)
        {

            power = ((dsBR.getDistance(DistanceUnit.CM)-desiredDistanceFromObj)/divisor)*multiplier;
            setPowerAll(power);
        }
        setPowerZero();
    }

}

