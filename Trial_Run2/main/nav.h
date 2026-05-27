#pragma once
bool executeLineFollow(int bSpeed, int maxPWM);
bool executeWallFollow(int bSpeed, int maxPWM, int mode);
bool isLineDetected(); // NEW: Used for returning to the track after bypass