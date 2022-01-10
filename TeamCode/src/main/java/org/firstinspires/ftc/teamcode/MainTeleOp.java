package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MainTele")

public class MainTeleOp extends HwMapIter{
    boolean armUpRangeIsValid, armDownRangeIsValid, buttonPressed=false, toggle = false;
    int downRange, upRange;
    double newPincerPos;
    @Override
    public void init()
    {
        initHwMap();
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
        if((Math.abs(gamepad1.left_stick_y)>.1)&&(Math.abs(gamepad1.right_stick_y)>.1) )
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
        else {
            setPowerZero();
        }
    //robot.game(win);
        //  controller two... arm/claw/carouspinner

        if(gamepad2.left_bumper) {
            carouSpin.setPower(1);
        }
        else if(gamepad2.right_bumper) {
            carouSpin.setPower(-1);
        }
        else{
            carouSpin.setPower(0);
        }

        //armUpRangeIsValid = arm.getCurrentPosition()<upRange;//change value
        //armDownRangeIsValid = arm.getCurrentPosition()>downRange;
        if(gamepad2.left_stick_y<0)//&&armDownRangeIsValid)
        {
            extend.setPower(1);
        }
        else if(gamepad2.left_stick_y>0)//&&armUpRangeIsValid)
        {
            extend.setPower(-1);
        }
        else{
            extend.setPower(0);
        }

        if(gamepad2.right_stick_y<0)
        {
            arm.setPower(.4);
        }
        else if(gamepad2.right_stick_y>0)
        {
            arm.setPower(-.4);
        }
        else{
            arm.setPower(0);
        }

        if(gamepad2.a && !toggle){
            if(pincer.getPosition() == 0){
                pincer.setPosition(1);
            }
            else{ pincer.setPosition(0); }
            toggle = true;
        }
        else if (!gamepad2.a){
            toggle = false;
        }

        telemetry.addData("current servo position: ", pincer.getPosition());
        telemetry.update();
        //telemetry.addData("encoder counts: ",arm.getCurrentPosition());
        //telemetry.addData("gamepad joystick val: ", gamepad2.left_stick_y);

    }
    @Override
    public void stop()
    {

    }
}
