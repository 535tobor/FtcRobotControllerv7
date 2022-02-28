package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MainTele")

public class MainTeleOp extends HwMapIter{
    boolean armRangeIsValid = false, extendRangeIsValid, buttonPressed=false, toggle = false;
    double curPincerPos = 0, currentAngle = 0;
    int curArmPos, currentEncoder;
    boolean encoderLock = false, angleLock = false;
    String fmtstr = "Angle to carousel: ";
    String angletxt = "";
    String encodertxt = "";
    public final double WHEEL_DIAMETER = 5.65, COUNTS_PER_INCH = 1120/(WHEEL_DIAMETER * 3.14);
    @Override
    public void init()
    {
        initHwMap();
        curPincerPos = pincer.getPosition();
        curArmPos = arm.getCurrentPosition();
        //extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        if(!encoderLock)
        {
            currentEncoder = (fl.getCurrentPosition()+fr.getCurrentPosition()+bl.getCurrentPosition()+br.getCurrentPosition())/4;
        }
        if(!angleLock)
        {
            currentAngle = getRoboAngle();
        }
        if(gamepad1.x)
        {
            angleLock = true;
            angletxt = String.valueOf(currentAngle - getRoboAngle());
        }
        if(gamepad1.y)
        {
            encoderLock = true;
            encodertxt = Math.abs(currentEncoder - ((fl.getCurrentPosition()+fr.getCurrentPosition()+bl.getCurrentPosition()+br.getCurrentPosition())/4))/COUNTS_PER_INCH + "\n";
        }
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
            fl.setPower(gamepad1.right_trigger*.35);
            bl.setPower(gamepad1.right_trigger*.35);
            fr.setPower(-gamepad1.right_trigger*.35);
            br.setPower(-gamepad1.right_trigger*.35);
        }
        else if(gamepad1.left_trigger>.1)
        {
            fl.setPower(-gamepad1.left_trigger*.35);
            bl.setPower(-gamepad1.left_trigger*.35);
            fr.setPower(gamepad1.left_trigger*.35);
            br.setPower(gamepad1.left_trigger*.35);
        }
//        else if (gamepad1.x)
//        {
//            fl.setPower(-1);
//            bl.setPower(-1);
//            fr.setPower(1);
//            br.setPower(1);
//        }
//        else if(gamepad1.b)
//        {
//            fl.setPower(1);
//            bl.setPower(1);
//            fr.setPower(-1);
//            br.setPower(-1);
//        }
        else if(gamepad1.dpad_up)
        {
            setPowerAll(.5);
        }
        else if(gamepad1.dpad_down)
        {
            setPowerAll(-.5);
        }
        else {

            setPowerZero();
        }

    //robot.game(win);

        //  controller two... arm/claw/carouspinner

        if(gamepad2.left_bumper) {
            carouSpin.setPower(.42);
        }
        else if(gamepad2.right_bumper) {
            carouSpin.setPower(-.55);
        }
        else{
            carouSpin.setPower(0);
        }
        if(gamepad2.left_stick_y<0&&extend.getCurrentPosition()<9000)
        {
            extend.setPower(1);
        }
        else if(gamepad2.left_stick_y>0&&extend.getCurrentPosition()>2000)
        {
            extend.setPower(-1);
        }
        else if(gamepad2.left_stick_y>0&&extend.getCurrentPosition()<2000&&extend.getCurrentPosition()>10)
        {
            extend.setPower(-.5);
        }
        else if(gamepad2.left_stick_y<0&&(extend.getCurrentPosition()<10400))
        {
            extend.setPower(.5);
        }
        else{
            extend.setPower(0);
        }
        if(gamepad2.right_stick_y<0)
        {
            arm.setPower(-.4);
        }
        else if(gamepad2.right_stick_y>0)
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
//        curPincerPos = Math.max(0, curPincerPos);
//        curPincerPos = Math.min(.45, curPincerPos);

        telemetry.addData(fmtstr, angletxt);
        telemetry.addData("Encoder Inches to position: ", encodertxt);
//        telemetry.addData("current servo position: ", pincer.getPosition());
//        telemetry.addData("arm encoder counts: ", arm.getCurrentPosition());
        //telemetry.addData("extend encoder counts: ", extend.getCurrentPosition());
//        telemetry.addData("Ds: ", dsL.getDistance(DistanceUnit.CM));
//        telemetry.addData("DsBR: ", dsBR.getDistance(DistanceUnit.CM));
        telemetry.update();

    }
    @Override
    public void stop()
    {

    }
}
