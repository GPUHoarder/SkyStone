package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Test Autonomous", group="LinearOpMode")

public class TrobotoixTestAutonomous extends LinearOpMode {




    double distance = 0;
    double actualSpeed = 0;
    double

    public void runOpMode() {
        Configure configure = new Configure(hardwareMap);
        configure.Initialize();
        Trobot trobot = configure.getTrobot();

        Drive drive = new Drive(trobot);




        waitForStart();

        drive.DriveForward(1);
        actualSpeed = speed*72.5;

    }
}
