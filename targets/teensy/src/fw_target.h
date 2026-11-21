// What the Teensy 4.1 target tells the firmware core. The build puts this in
// front of every source file (see platformio.ini), so the core itself
// holds nothing that is particular to one board.
#pragma once

// The core's large, seldom-read buffers go in RAM2, leaving the fast RAM
// for code, the tables read most often and the display buffer. RAM2 is not
// cleared at start-up; the core initialises everything it puts there.
#define FW_BIGMEM __attribute__((section(".dmabuffers"), used))
