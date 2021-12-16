package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "4WLDR")

public class FourWLDriv extends HwMapIter{
    boolean armUpRangeIsValid, armDownRangeIsValid;
    int downRange, upRange;
    double pincerOpen = 0, pincerClosed = 1;
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
        if(Math.abs(gamepad1.left_stick_y)>.1)
        {
            fl.setPower(-gamepad1.left_stick_y);
            bl.setPower(-gamepad1.left_stick_y);
        }
        else {
            fl.setPower(0);
            bl.setPower(0);
        }
        if(Math.abs(gamepad1.right_stick_y)>.1) {
            fr.setPower(-gamepad1.right_stick_y);
            br.setPower(-gamepad1.right_stick_y);
        }
        else {
            fr.setPower(0);
            br.setPower(0);
        }

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
            arm.setPower(-1);
        }
        else if(gamepad2.left_stick_y>0)//&&armUpRangeIsValid)
        {
            arm.setPower(1);
        }
        else{
            arm.setPower(0);
        }

        if(gamepad2.right_stick_y<0)
        {
            extend.setPower(-.2);
        }
        else if(gamepad2.right_stick_y>0)
        {
            extend.setPower(.2);
        }
        else{
            extend.setPower(0);
        }

        if(gamepad2.a)
        {
            pincer.setPosition(pincerOpen);
        }
        else if(gamepad2.b)
        {
            pincer.setPosition(pincerClosed);
        }
        //telemetry.addData("current servo position: ", pincer.getPosition());
        //telemetry.addData("encoder counts: ",arm.getCurrentPosition());
        //telemetry.addData("gamepad joystick val: ", gamepad2.left_stick_y);

    }
    @Override
    public void stop()
    {

    }
}
