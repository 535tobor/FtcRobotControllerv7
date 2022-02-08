package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "MainTele")

public class MainTeleOp extends HwMapIter{
    boolean armRangeIsValid = false, extendRangeIsValid, buttonPressed=false, toggle = false;
    double curPincerPos = 0;
    int curArmPos;
    @Override
    public void init()
    {
        initHwMap();
        curPincerPos = pincer.getPosition();
        curArmPos = arm.getCurrentPosition();
        //telemetry.addData("servo: ", pincer.getPosition());
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
        if((Math.abs(gamepad1.left_stick_y)>.1)||Math.abs(gamepad1.right_stick_y)>.1)
        {
            fl.setPower(-gamepad1.left_stick_y);
            bl.setPower(-gamepad1.left_stick_y);
            fr.setPower(-gamepad1.right_stick_y);
            br.setPower(-gamepad1.right_stick_y);
        }
        else if(gamepad1.right_trigger>.1)
        {
            fl.setPower(gamepad1.right_trigger*.5);
            bl.setPower(gamepad1.right_trigger*.5);
            fr.setPower(-gamepad1.right_trigger*.5);
            br.setPower(-gamepad1.right_trigger*.5);
        }
        else if(gamepad1.left_trigger>.1)
        {
            fl.setPower(-gamepad1.left_trigger*.5);
            bl.setPower(-gamepad1.left_trigger*.5);
            fr.setPower(gamepad1.left_trigger*.5);
            br.setPower(gamepad1.left_trigger*.5);
        }
        else if (gamepad1.x)
        {
            fl.setPower(-1);
            bl.setPower(-1);
            fr.setPower(1);
            br.setPower(1);
        }
        else if(gamepad1.b)
        {
            fl.setPower(1);
            bl.setPower(1);
            fr.setPower(-1);
            br.setPower(-1);
        }
        else {

            setPowerZero();
        }

    //robot.game(win);

        //  controller two... arm/claw/carouspinner

        if(gamepad2.left_bumper) {
            carouSpin.setPower(.3);
        }
        else if(gamepad2.right_bumper) {
            carouSpin.setPower(-.3);
        }
        else{
            carouSpin.setPower(0);
        }
        if(gamepad2.left_stick_y<0)//&&extend.getCurrentPosition()<15600)
        {
            extend.setPower(1);
        }
        else if(gamepad2.left_stick_y>0)//&&extend.getCurrentPosition()>0)
        {
            extend.setPower(-1);
        }
        else{
            extend.setPower(0);
        }
        if(gamepad2.right_stick_y<0)//&&arm.getCurrentPosition()>0)
        {
            arm.setPower(-.4);
        }
        else if(gamepad2.right_stick_y>0)//&&arm.getCurrentPosition()<1555)
        {
            arm.setPower(.4);
        }
        else{
            arm.setPower(0);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
//        if(gamepad2.a && !toggle){
//            if(pincer.getPosition() == 0){
//                pincer.setPosition(1);
//            }
//            else{ pincer.setPosition(0); }
//            toggle = true;
//        }
//        else if (!gamepad2.a){
//            toggle = false;
//        }

        if(gamepad2.a)
        {
            curPincerPos+=.014;//.005;
            pincer.setPosition(curPincerPos);
        }
        else if(gamepad2.b)
        {
            curPincerPos-=.014;//.005;
            pincer.setPosition(curPincerPos);
        }
        curPincerPos = Math.max(0, curPincerPos);
        curPincerPos = Math.min(.375, curPincerPos);

        telemetry.addData("current servo position: ", pincer.getPosition());
        //telemetry.addData("arm encoder counts: ", arm.getCurrentPosition());
        //telemetry.addData("extend encoder counts: ", extend.getCurrentPosition());
        telemetry.addData("Ds: ", ds.getDistance(DistanceUnit.CM));
        telemetry.update();

    }
    @Override
    public void stop()
    {

    }
}
