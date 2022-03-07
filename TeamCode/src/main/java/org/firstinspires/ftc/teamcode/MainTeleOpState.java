package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "MainTeleState")

public class MainTeleOpState extends HwMapIterState{
    boolean intakeCanToggle = true, intakeToggle = false, reverseIntakeCanToggle = true, reverseIntakeToggle = false;
    boolean basketCanToggle = true, basketToggle = false, distanceThreshold = false;
    double turnPower = 1, initTime = 0, initTimeR = 0;
    boolean startBasketLOpen = false, startBasketROpen = false;
    ElapsedTime timer = new ElapsedTime();
    public final double WHEEL_DIAMETER = 5.65, COUNTS_PER_INCH = 1120/(WHEEL_DIAMETER * 3.14);
    @Override
    public void init()
    {
        initHwMap();
        updateGyro();
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
    if(gamepad2.a){
        basket.setPosition((basket.getPosition()) + .001);
    }
    else if(gamepad2.b){
        basket.setPosition((basket.getPosition()) - .001);
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
    // BUCKET CONTROL
        if(gamepad2.right_bumper) {  basket.setPosition(.3); }
        else if(gamepad2.left_bumper) { basket.setPosition(0); }
        else  {basket.setPosition(.15); }
    // TURN EXTENDER
        /**
        if((gamepad2.right_stick_x < 0)&&distanceThreshold(9,17)) { turnExtender.setPower(1); }// change threshold to handle overshooting; clockwise to the left
        else if((gamepad2.right_stick_x > 0)&&distanceThreshold(9,17)) { turnExtender.setPower(-1); }//counterclockwise to the right
        else if((gamepad2.right_stick_x == 0)&&distanceThreshold(4,9))
        {
            turnPower = (dsTurn.getDistance(DistanceUnit.CM)-4)/10;
            if(turnPower<.06) { turnPower = 0; }
            turnExtender.setPower(turnPower);//positive
        }
        else if((gamepad2.right_stick_x > 0)&&distanceThreshold(17,22))
        {
            turnPower = (22-dsTurn.getDistance(DistanceUnit.CM))/10;
            if(turnPower<.06) { turnPower = 0; }
            turnExtender.setPower(-turnPower);//negative
        }
         **/
        if(gamepad2.right_stick_x < 0) {turnExtender.setPower(.8);}
        else if (gamepad2.right_stick_x > 0) {turnExtender.setPower(-.8);}
        else {turnExtender.setPower(0);};
    // EXTENDER
        extender.setPower(gamepad2.left_stick_y);
    // BLOCK INTAKE/OUTAKE
        if((gamepad2.right_trigger > .1)&&intakeCanToggle) {
            intakeCanToggle=false;
            if(intakeToggle) { launchSetZero(); intakeToggle=false; }
            else{ launch(); intakeToggle=true; }
        }
        else { intakeCanToggle=true; }

        if((gamepad2.left_trigger > .1)&& reverseIntakeCanToggle)
        {
            reverseIntakeCanToggle=false;
            if(reverseIntakeToggle){ launchSetZero(); reverseIntakeToggle=false; }
            else{ reverseLaunch();  reverseIntakeToggle=true; }
        }
        else { reverseIntakeCanToggle = true; }


        if(!startBasketLOpen)
        {
            startBasketLOpen = gamepad2.left_bumper;
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
                basket.setPosition(.15);
                startBasketLOpen = false;
            }

        }

        if(!startBasketROpen)
        {
            startBasketROpen = gamepad2.right_bumper;
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
                basket.setPosition(.15);
                startBasketROpen = false;
            }

        }



        telemetry.addData("distance: ", dsTurn.getDistance(DistanceUnit.CM));
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
}
