package org.firstinspires.ftc.teamcode.decode.control.controller;

import org.firstinspires.ftc.teamcode.decode.control.motion.State;

public interface FeedbackController extends Controller {
    double calculate(State measurement);
}
