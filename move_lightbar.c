/*
    This file contains the process of moving the lightbar either out or in.
    It will take environment variables to determine the profile of the motion.
    Ie. the max speed, max acceleration, jerk, initial velocity, and gear ratio.
    It will use the contents of stepcount.dat to determine the cur position of the lightbar.
    If stepcount.dat is 0 then it moves 45 deg clockwise, if it is close to MAX_MSCNT then it moves 45 deg counter-clockwise.
    If it's in the middle of the motion then it will continue in the same direction, or default to close if that is not known.

*/