package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MainTele")

public class MainTeleOpState extends HwMapIterState{
    boolean intakeCanToggle = true, intakeToggle = false, reverseIntakeCanToggle = true, reverseIntakeToggle = false;
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

        if(gamepad2.x)
        {
            if(intakeCanToggle) //make sure that the code doesn't just toggle the thing every iteration as long as the x is held
            {
                intakeCanToggle=false;
                //if the launcher is currently running, run this code to turn it off:
                if(intakeToggle)
                {
                    launchSetZero(); //turn off the launcher motor
                    intakeToggle=false; //remember that the launcher motor has been turned off
                }
                //if the launcher isn't currently running, run this code to turn it on:
                else
                {
                    launch(); //turn on the launcher motor
                    intakeToggle=true; //remember that the launcher motor has been turned on
                }
            }
        }
        else
        {
            intakeCanToggle=true;
        }
        if( gamepad2.y)
        {
            if(reverseIntakeCanToggle) //make sure that the code doesn't just toggle the thing every iteration as long as the x is held
            {
                reverseIntakeCanToggle=false;
                //if the launcher is currently running, run this code to turn it off:
                if(reverseIntakeToggle)
                {
                    launchSetZero(); //turn off the launcher motor
                    reverseIntakeToggle=false; //remember that the launcher motor has been turned off
                }
                //if the launcher isn't currently running, run this code to turn it on:
                else
                {
                    reverseLaunch(); //turn on the launcher motor
                    reverseIntakeToggle=true; //remember that the launcher motor has been turned on
                }
            }
        }
        else
        {
            reverseIntakeCanToggle = true;
        }

        telemetry.update();

    }
    @Override
    public void stop()
    {

    }
}
