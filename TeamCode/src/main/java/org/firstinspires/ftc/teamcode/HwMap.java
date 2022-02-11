package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class HwMap extends LinearOpMode {

    public DcMotor fl, fr, br, bl,arm , extend, carouSpin;
    public Servo pincer;
    public DistanceSensor dsL, dsR;

    public int rotations = 0;
    public BNO055IMU imu;
    public Orientation gyroAngles;
    public double desiredRobotHeading;
    public final double WHEEL_DIA = 5.65, COUNTS_PER_INCH = 1120/(WHEEL_DIA * 3.14);
    public ElapsedTime runtime = new ElapsedTime();
    public String side = "empty", level = "empty";

    public void initHwMap()
    {
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        carouSpin = hardwareMap.dcMotor.get("carousel");
        arm = hardwareMap.dcMotor.get("arm");
        extend = hardwareMap.dcMotor.get("extender");
        pincer = hardwareMap.servo.get("pincer");
        dsL = hardwareMap.get(DistanceSensor.class, "ds"); //left
        dsR = hardwareMap.get(DistanceSensor.class, "ds2"); //right

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    @Override
    public void runOpMode()  {}

    public void pincerGrip()
    {
        pincer.setPosition(.375);
    }
    public void pincerRelease()
    {
        pincer.setPosition(0);
    }
    public void spin(double time, double power)
    {
        runtime.reset();
        while (opModeIsActive()&& runtime.time() < time){
            carouSpin.setPower(power);
        }
        carouSpin.setPower(0);
    }

    public void updateGyro()
    {
        gyroAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public double getRobotAngle() {
        if (desiredRobotHeading - (rotations * 360 + gyroAngles.firstAngle) > 200) {
            rotations++;
        } else if (desiredRobotHeading - (rotations * 360 + gyroAngles.firstAngle) < -200) {
            rotations--;
        }
        return (rotations * 360 + gyroAngles.firstAngle);
    }
    public void setModeAll(DcMotor.RunMode mode)
    {
        fl.setMode(mode); fr.setMode(mode); bl.setMode(mode); br.setMode(mode);
    }
    public void setPowerLeft(double power)
    {
        fl.setPower(power); bl.setPower(power);
    }
    public void setPowerRight(double power)
    {
        fr.setPower(power); br.setPower(power);
    }
    public void setPowerAll(double power)
    {
        setPowerLeft(power); setPowerRight(power);
    }
    public void setPowerZero()
    {
        setPowerAll(0);
    }

    public void driveByTime(double speed, double time){
        runtime.reset();
        while (opModeIsActive() && runtime.time() < time ){
            setPowerAll(speed);
        }
        setPowerZero();
    }

    public void robotTurn(int degrees, double timeoutS) //from current
    {

        double initAngle = getRobotAngle();
        double desiredAngle = initAngle + degrees;
        runtime.reset();
        while(opModeIsActive()&&(runtime.time()<timeoutS)&&(getRobotAngle()>( desiredAngle + 2)|| getRobotAngle()<(desiredAngle- 2)))
        {//left is pos
            updateGyro();
            if(degrees>0)
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
        setPowerAll(0);
    }
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
            boolean why = (runtime.seconds() < timeoutS);
            telemetry.addData("time bool: "+why+ " cur Pos bool: ",tele);
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
    public void encoderDriveAndMoveArm(double Lspeed, double Rspeed, double Inches, double timeoutS, double rampup){
        int newLeftTarget = (fl.getCurrentPosition() + bl.getCurrentPosition() )/2 + (int)(Inches * COUNTS_PER_INCH);
        int newRightTarget= (fr.getCurrentPosition() + br.getCurrentPosition() )/2 + (int)(Inches * COUNTS_PER_INCH);
        boolean done = false;
        runtime.reset();
        while ( opModeIsActive() && (runtime.seconds() < timeoutS) && (Math.abs(fl.getCurrentPosition() + bl.getCurrentPosition()) /2 < newLeftTarget  &&Math.abs(fr.getCurrentPosition() + br.getCurrentPosition())/2 < newRightTarget))
        {
            double rem = (Math.abs(fl.getCurrentPosition())+ Math.abs(bl.getCurrentPosition())+Math.abs(fr.getCurrentPosition()) + Math.abs(br.getCurrentPosition()))/4;
            double NLspeed, NRspeed;
            boolean tele = (Math.abs(fl.getCurrentPosition() + bl.getCurrentPosition()) /2 < newLeftTarget  && Math.abs(fr.getCurrentPosition() + br.getCurrentPosition())/2 < newRightTarget);
            telemetry.update();

            if(!arm.isBusy()&&!done)
            {
                armBrake();
                done = true;
            }

            double R = runtime.seconds();
            if (R < rampup) {
                double ramp = R / rampup; NLspeed = Lspeed * ramp; NRspeed = Rspeed * ramp;
            }
            else if(rem > (2000) )
            {
                NLspeed = Lspeed; NRspeed = Rspeed;
            }
            else if(rem > (400) && (Lspeed*.2) > .1 && (Rspeed*.2) > .1) {
                NLspeed = Lspeed * (rem / 2000); NRspeed = Rspeed * (rem / 2000);
            }
            else {
                NLspeed = Lspeed * .2; NRspeed = Rspeed * .2;
            }
            setPowerLeft(NLspeed);
            setPowerRight(NRspeed);
        }
        setPowerZero();
    }


    public boolean threshold(int val, int otherval, int range)
    {
        return (val>( otherval + range)||val<( otherval - range));
    }
    public boolean threshold(double val, double otherval, double range)
    {
        return (val>( otherval + range)||val<( otherval - range));
    }

    public void runEncoder(DcMotor motor, int desiredCounts,  int thresholdRange, double power)
    {
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

    public void driveByDistance( double desired, double multiplier)
    {
        double divisor = desired - dsL.getDistance(DistanceUnit.CM); //desired - initial
        double power = .06;//start power greater than .055 so it complies with the while loop; algorithm c=
        multiplier = Math.max(0.1, multiplier);
        multiplier = Math.min(1, multiplier);
        while(opModeIsActive()&&threshold(dsL.getDistance(DistanceUnit.CM), desired, .3) && Math.abs(power)>.055)
        {
            power = ((dsL.getDistance(DistanceUnit.CM)-desired)/divisor)*multiplier;
            setPowerAll(power);
        }
        setPowerZero();
    }

    public void setPincerPos(double position, double timeout)
    {
        double curPincerPos = pincer.getPosition();
        int multiplier = 1;
        if(curPincerPos>position)
        {
            multiplier = -1;
        }
        runtime.reset();
        while(opModeIsActive()&&threshold(curPincerPos, position, .01)&&runtime.time()<timeout)
        {
            curPincerPos+=(multiplier*.014);
            pincer.setPosition(curPincerPos);
        }
    }

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
    public void armMoveAndIdle(int counts, double power)
    {
        motorRTPIdle(arm, counts, power);
    }
    public void motorRTP(DcMotor motor, int counts, double power)
    {
        motor.setTargetPosition(counts);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }
    public void startExtend(int counts, double power)
    {
        motorRTP(extend, extend.getCurrentPosition() + counts, power);
    }
    public void startArmMovement(int counts, double power)
    {
        motorRTP(arm, arm.getCurrentPosition()+ counts, power);
    }
    public void deliverBox(int degreesToPark, int inchesToBackUp)
    {
        armMoveAndIdle(arm.getCurrentPosition()+850, .4); //lower arm to chassis base
        waitFor(1);
        pincerGrip();
        waitFor(1);
        startArmMovement( arm.getCurrentPosition()-1250, -.4); //start raise arm
        runtime.reset();
        while(opModeIsActive() && runtime.time()<1) //distract flow to give time to start extend arm
        {
            if(!arm.isBusy())
            {
                armBrake();
            }
        }
        startExtend( 13500, .7); //start extend arm
        encoderDriveAndMoveArm(.7, .7, 19, 5, 1); //drive to tower
        while(opModeIsActive()&& (arm.isBusy()||extend.isBusy()))
        {
            telemetry.addData("Waiting for motors", "");
        }
        armBrake();
        waitFor(1);
        pincerRelease();
        waitFor(1);
        startExtend(-10000, -.8);
        encoderDrive(-.7, -.7, 3, 5, 1);
        robotTurn(degreesToPark, 4);
        encoderDrive(-.7, -.7, inchesToBackUp, 8, 1);
    }

    public void barcodeDetect() {
        if (side.equals("left")) {
            if (dsL.getDistance(DistanceUnit.CM) < 60) {
                level =  "Middle";
            } else if (dsR.getDistance(DistanceUnit.CM) < 60) {
                level = "Bottom";
            } else {
                level = "Top";
            }
            // left         right
            // _  _  _   O  _  _  _
            //    ^  ^      ^  ^
            //    L  R      L  R
            //   Mid Bot   Bot Mid 
        } else {
            if (dsL.getDistance(DistanceUnit.CM) < 60) {
                level = "Bottom";
            } else if (dsR.getDistance(DistanceUnit.CM) < 60) {
                level = "Middle";
            } else {
                level = "Top";
            }

        }
        telemetry.addData("Level: ", level);
    }
    public void waitFor(double seconds)
    {
        runtime.reset();
        while(opModeIsActive()&& runtime.time()<seconds)
        {
            telemetry.update();
        }
    }
    public void armBrake()
    {
        arm.setPower(0);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}

