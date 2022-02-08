package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "DS")
@Disabled
public class DistanceSensorTest extends HwMap{
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode()
    {
        initHwMap();
        waitForStart();
        if(opModeIsActive())
        {

            while(opModeIsActive()&&threshold(ds.getDistance(DistanceUnit.CM), 11.9, .3)&&timer.time()<3)
            {
                arm.setPower(.4);
            }
            arm.setPower(0);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

}
