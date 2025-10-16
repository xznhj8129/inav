Welcome, agent.

If you are reading this, your mission is to perform the long-requested adaptation of Bidirectional D-Shot ESC telemetry to INAV. To this end, you will find Betaflight's latest source code under the bfsrc/ directory. Consider bfsrc/ to be read-only. The main files of interest are:
* bfsrc/main/drivers/dshot_bitbang_decode.c
* bfsrc/main/drivers/dshot_bitbang_decode.h
* bfsrc/main/drivers/dshot_bitbang.h
* bfsrc/main/drivers/dshot.c
* bfsrc/main/drivers/dshot_command.c
* bfsrc/main/drivers/dshot_command.h
* bfsrc/main/drivers/dshot.h

In addition to copying these files to INAV's main directory, you must review how they are used within Betaflight to proprely understand their usage and where they fit into INAV, and what modifications to apply to them or other files to proprely bring Bidirectional DShot ESC telemetry to INAV. 

Good luck and godspeed.