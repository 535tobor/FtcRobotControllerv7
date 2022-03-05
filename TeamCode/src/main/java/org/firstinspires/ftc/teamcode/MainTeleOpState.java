package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "MainTeleState")

public class MainTeleOpState extends HwMapIterState{
    boolean intakeCanToggle = true, intakeToggle = false, reverseIntakeCanToggle = true, reverseIntakeToggle = false;
    boolean basketCanToggle = true, basketToggle = false, distanceThreshold = false;
    double turnPower = 1, initTime = 0;
    boolean startAutoMov = false;
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

    //robot.game(win);

        //  controller two... arm/claw/carouspinner

        if(gamepad2.dpad_left) {
            carouSpin.setPower(.42);
        } //carousel spinner left
        else if(gamepad2.dpad_right) {
            carouSpin.setPower(-.42);
        } // carousel spinner right
        else{
            carouSpin.setPower(0);
        }
        // also this

        if(gamepad2.right_bumper) {  basket.setPosition(.3); }
        else if(gamepad2.left_bumper) { basket.setPosition(0); }
        else  {basket.setPosition(.15); }
        // Change because it wont work
        if((gamepad2.right_stick_x < 0)&&distanceThreshold(9,17)) { turnExtender.setPower(1); }// change threshold to handle overshooting; clockwise to the left
        else if((gamepad2.right_stick_x > 0)&&distanceThreshold(9,17)) { turnExtender.setPower(-1); }//counterclockwise to the right
        else if((gamepad2.right_stick_x < 0)&&distanceThreshold(4,9))
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

        extender.setPower(gamepad2.left_stick_y);

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


        if(!startAutoMov)
        {
            startAutoMov = gamepad2.right_stick_button;
            initTime = getRuntime();
        }
        else
        {
            if(getRuntime()<initTime+3)
            {
                extender.setPower(.3);
            }
            else
            {
                extender.setPower(0);
                startAutoMov = false;
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
