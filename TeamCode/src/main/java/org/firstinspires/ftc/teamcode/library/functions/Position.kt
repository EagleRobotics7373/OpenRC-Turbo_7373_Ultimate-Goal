package org.firstinspires.ftc.teamcode.library.functions

enum class Position {
    LEFT, CENTER, RIGHT, NULL;

    fun flip() : Position {
        return when(this) {
            LEFT   -> RIGHT
            RIGHT  -> LEFT
            CENTER -> CENTER
            NULL   -> NULL
        }
    }
}