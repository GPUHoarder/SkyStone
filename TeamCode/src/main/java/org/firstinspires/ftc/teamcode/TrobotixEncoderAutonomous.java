package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Encoder Autonomous", group="Autonomous")

public class TrobotixEncoderAutonomous extends LinearOpMode {
    public void runOpMode() {
        Configure configure = new Configure(hardwareMap);
        configure.initialize();
        Trobot trobot = configure.getTrobot();

        Drive drive = new Drive(trobot);

        drive.encoderDrive(0.75, 30);
        sleep(3000);
        drive.encoderDrive(0.75, -30);
        waitForStart();
    }
}
