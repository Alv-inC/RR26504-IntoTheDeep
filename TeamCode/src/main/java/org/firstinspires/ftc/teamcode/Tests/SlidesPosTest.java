package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Slides;

@TeleOp(name = "Slides Position Test", group = "Testing")
public class SlidesPosTest extends com.qualcomm.robotcore.eventloop.opmode.OpMode {
    private Slides slides;

    @Override
    public void init() {
        slides = new Slides(hardwareMap);
        slides.initialize();

        telemetry.addLine("Slides Test Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Extend slides
        if (gamepad1.dpad_up) {
            slides.moveSlides(Slides.SLIDES_EXTENDED);
            telemetry.addLine("Slides moving to extended position");
        }
        // Retract slides
        else if (gamepad1.dpad_down) {
            slides.moveSlides(Slides.SLIDES_RETRACTED);
            telemetry.addLine("Slides moving to retracted position");
        }
        // Reset encoders
        else if (gamepad1.b) {
            slides.resetSlides();
            telemetry.addLine("Slides reset");
        }


    }
}
