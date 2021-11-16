package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "4WLDR")
//@Disabled
public class FourWLDriv extends HwMapIter{
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
        long ticCount = fl.getCurrentPosition();
        telemetry.addData("Tic Count: ", ticCount);
        if(Math.abs(gamepad1.left_stick_y)>.1) {
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
        /**
        //I just set up everything below this to see how it worked, if it shouldn't be here just delete it
        if(gamepad1.a) {
            servo.setPosition(0.5); //just remembering how to code, this isn't actually gonna be used probably
        }
        if(gamepad1.b) {

        }
        if(gamepad1.x) {

        }
        if(gamepad1.y) {

        }
        if(gamepad1.dpad_up) {

        }
        if(gamepad1.dpad_down) {

        }
        if(gamepad1.dpad_left) {

        }
        if(gamepad1.dpad_right) {

        }
        if(gamepad1.left_bumper) {

        }
        if(gamepad1.right_bumper) {

        }
        // controller two...
        if(Math.abs(gamepad2.left_stick_y)>.1) {

        }
        else {

        }
        if(Math.abs(gamepad2.right_stick_y)>.1) {

        }
        else {

        }

        if(gamepad2.a) {

        }
        if(gamepad2.b) {

        }
        if(gamepad2.x) {

        }
        if(gamepad2.y) {

        }
        if(gamepad2.dpad_up) {

        }
        if(gamepad2.dpad_down) {

        }
        if(gamepad2.dpad_left) {

        }
        if(gamepad2.dpad_right) {

        }
        if(gamepad2.left_bumper) {

        }
        if(gamepad2.right_bumper) {

        }
    }
    **/
    @Override
    public void stop()
    {

    }
}
