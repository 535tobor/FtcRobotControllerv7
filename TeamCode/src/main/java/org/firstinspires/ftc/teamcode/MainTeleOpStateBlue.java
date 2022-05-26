package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "MainTeleStateBlue")

public class MainTeleOpStateBlue extends HwMapIterState{
    boolean intakeCanToggle = true, intakeToggle = false, reverseIntakeCanToggle = true, reverseIntakeToggle = false;
    boolean basketCanToggle = true, basketToggle = false, distanceThreshold = false;
    double turnPower = 1, initTime = 0, initTimeR;
    int desiredExt = 0, initExt = 0;
    boolean startBasketLOpen = false, startBasketROpen = false, startArmExtend = false;
    boolean startRetract = false;
    int max = 0;
    ElapsedTime timer = new ElapsedTime();
    public final double WHEEL_DIAMETER = 5.65, COUNTS_PER_INCH = 1120/(WHEEL_DIAMETER * 3.14);



    @Override
    public void init()
    {
        initHwMap();
        updateGyro();
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    @Override
    public void init_loop()
    {

    }
    @Override
    public void start()
    {

    }
    @Override
    public void loop()
    {
        extendThreshold = extender.getCurrentPosition()>0&&extender.getCurrentPosition()<maxExtend;
        // controller one... driving
        if((Math.abs(gamepad1.left_stick_y)>.1)||Math.abs(gamepad1.right_stick_y)>.1) //drive with both joysticks
        {
            setPowerLeft(-gamepad1.left_stick_y);
            setPowerRight(-gamepad1.right_stick_y);
        }
        else if(gamepad1.right_trigger>.1) //slow turn left
        {
            setPowerLeft(gamepad1.right_trigger*.35);
            setPowerRight(-gamepad1.right_trigger*.35);
        }
        else if(gamepad1.left_trigger>.1)// slow turn right
        {
            setPowerLeft(-gamepad1.left_trigger*.35);
            setPowerRight(gamepad1.left_trigger*.35);
        }
        else if(gamepad1.dpad_up) //slow drive forwards
        {
            setPowerAll(.5);
        }
        else if(gamepad1.dpad_down) // slow drive backwards
        {
            setPowerAll(-.5);
        }
        else {

            setPowerZero();
        }
    //robot.game(win);

        //  controller two... arm/claw/carouspinner
    // CAROUSEL SPINNER
        if(gamepad2.dpad_left) {
            carouSpin.setPower(.42);
        } //carousel spinner left
        else if(gamepad2.dpad_right) {
            carouSpin.setPower(-.42);
        } // carousel spinner right
        else{
            carouSpin.setPower(0);
        }
    // TURN EXTENDER
        if(gamepad2.right_stick_x > 0) {turnExtender.setPower(-.8);}
        else if (gamepad2.right_stick_x < 0) {turnExtender.setPower(.8);}
        else if(!startLTurn&&!startRTurn){turnExtender.setPower(0);}
    // EXTENDER
        if(!startArmExtend) { extender.setPower(gamepad2.left_stick_y);}
        if(extender.getPower()==0)
        {
            extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    // BLOCK INTAKE/OUTAKE
        if(gamepad2.right_trigger > .01)
        {
            launch();
        }
        else if(gamepad2.left_trigger > .01)
        {
            reverseLaunch();
        }
        else
        {
            launchSetZero();
        }
        if(!startBasketLOpen)
        {
            startBasketLOpen = gamepad2.right_bumper;
            initTime = getRuntime(); // the time at which the movement starts
        }
        else // auto movement has started
        {
            if(getRuntime()<initTime+1)//checking if the correct amount of time has passed since movement has started
            {
                basket.setPosition(.3);
            }
            else
            {
                basket.setPosition(.125);
                startBasketLOpen = false;
            }

        }
        telemetry.addData("current turn power: ", turnPower);

        if(!startBasketROpen)
        {
            startBasketROpen = gamepad2.left_bumper;
            initTimeR = getRuntime(); // the time at which the movement starts
        }
        else // auto movement has started
        {
            if(getRuntime()<initTimeR+1)//checking if the correct amount of time has passed since movement has started
            {
                basket.setPosition(0);
            }
            else
            {
                basket.setPosition(.125);
                startBasketROpen = false;
            }

        }
        telemetry.addData("distance: ", dsTurn.getDistance(DistanceUnit.CM));
        telemetry.addData("power: ", turnExtender.getPower());
        telemetry.addData("encoder: ", extender.getCurrentPosition());
        telemetry.addData("encoder boolean: ", extendThreshold);
        telemetry.update();

    }
    @Override
    public void stop()
    {

    }

    public boolean distanceThreshold(double distanceLBound, double distanceRBound)
    {
        return(dsTurn.getDistance(DistanceUnit.CM)>distanceLBound&&dsTurn.getDistance(DistanceUnit.CM)<distanceRBound);
    }

    boolean startRAutoMov = false, startLAutoMov = false, startRTurn = false, startLTurn = false;
    double initRTime = 0, initLTime = 0, divisorDs = 0;
    final double dsLMost = 4;
    final double dsRMost = 22;
    final double dsLSafe = 9; //value = distance away from sensor where the basket will not hit the left side of the robot
    final double dsRSafe = 14; // value = distance away from sensor where the basket will not hit the right side of the robot
    final double maxExtend = 1420;
    final double tenPercentPowerCts = maxExtend*.9;
    boolean extendThreshold = true;
    public void macros()
    {
//                if(!startBasketROpen) {
//            if(!startRAutoMov) {  startBasketROpen = gamepad2.right_bumper; }
//            initRTime = getRuntime();
//        }
//        else {
//            if(getRuntime()<initRTime+1.5) { basket.setPosition(0); }
//            else { basket.setPosition(.15); startBasketROpen = false; }
//        }
//
//        //turn right (perspective from back of the robot)
//        if(!startRAutoMov) {
//            startRAutoMov = gamepad2.b;
//            if(startRAutoMov) { startArmExtend = true; startRTurn = true; }
//        }
//        else {
//            startRAutoMov = extender.isBusy()||turnExtender.getPower()!=0;
//            if(!startRAutoMov) { startBasketROpen = true; } // drop block after fully extending and turning to the right
//        }
//
//
//        if(!startArmExtend) { desiredExt = extender.getCurrentPosition()+2000; }
//        else{
//            if(distanceThreshold(dsLSafe, dsRSafe)) {
//                if (extender.getCurrentPosition() < desiredExt) {
//                    if (desiredExt - extender.getCurrentPosition() <= 210) {
//                        extender.setPower(1);
//                    } else {
//                        extender.setPower(((double) (desiredExt - extender.getCurrentPosition()) / 2000) + .1);
//                    }
//                } else {
//                    extender.setPower(0);
//                    startArmExtend = false;
//                }
//            }
//        }
//        if(!startRTurn){
//            divisorDs = dsRMost-dsTurn.getDistance(DistanceUnit.CM);//condition checker variable (init time, init position, etc
//        }
//        else{
//            if(dsTurn.getDistance(DistanceUnit.CM)<dsRMost) { turnExtender.setPower((dsTurn.getDistance(DistanceUnit.CM)-dsRMost)/divisorDs);}
//            else{ turnExtender.setPower(0); startRTurn = false;}
//        }
//
//
//
//
//
//
//
//        //left open basket
//        if(!startBasketLOpen) {
//            if(!startLAutoMov) {  startBasketLOpen = gamepad2.left_bumper; }
//            initLTime = getRuntime();
//        }
//        else {
//            if(getRuntime()<initLTime+1.5) { basket.setPosition(.3); }
//            else { basket.setPosition(.125); startBasketLOpen = false; }
//        }
//
//        //turn left (perspective from back of the robot)
//        if(!startLAutoMov) {
//            startLAutoMov = gamepad2.x;
//            if(startLAutoMov) { startArmExtend = true; startLTurn = true; }
//        }
//        else {
//            startLAutoMov = extender.isBusy()||turnExtender.getPower()!=0;
//            if(!startLAutoMov) { startBasketLOpen = true; }
//        }
//
//        if(!startLTurn){
//            divisorDs = dsLMost-dsTurn.getDistance(DistanceUnit.CM);//condition checker variable (init time, init position, etc
//        }
//        else{
//            if(dsTurn.getDistance(DistanceUnit.CM)>dsLMost) { turnExtender.setPower((dsTurn.getDistance(DistanceUnit.CM)-dsLMost)/divisorDs);}
//            else{ turnExtender.setPower(0); startLTurn = false;}
//        }
//
//
//
//
//        if(startArmExtend && distanceThreshold(dsLSafe, dsRSafe)) {
//            if (extender.getCurrentPosition() < maxExtend) {
//                if (extender.getCurrentPosition() >=tenPercentPowerCts) {
//                    extender.setPower(.1);
//                } else {
//                    extender.setPower(1-(extender.getCurrentPosition()/maxExtend));
//                }
//            } else {
//                extender.setPower(0);
//                startArmExtend = false;
//            }
//        }
//
//        if(Math.abs(gamepad2.left_stick_y)>.1 && extendThreshold)
//        {
//            extender.setPower(gamepad2.left_stick_y);
//        }
//        else if(!startArmExtend)
//        {
//            extender.setPower(0);
//        }
//
//        if(!startRetract)
//        {
//            startRetract = gamepad2.back;
//            if(extender.getMode() == DcMotor.RunMode.RUN_TO_POSITION&&!extender.isBusy())
//            {
//                extender.setPower(0);
//                extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            }
//        }
//        else
//        {
//            extender.setTargetPosition(50);
//            extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            extender.setPower(.8);
//            startRetract = false;
//        }
    }
}
