#ifndef BASICSTUFF_H
#define BASICSTUFF_H

//Arm
#define FLIGHTBOT	1 //Uncomment = Flight 

#define ARMSPEED_HIGH       35.0
#define ARMSPEED_MEDIUM     20.0
#define ARMSPEED_LOW        5.0

#define T_CCW   1
#define T_CW    0

#define T_CCW   1
#define T_CW    0

#ifdef FLIGHTBOT

//*******************************************Flight bot********************************************
#define A_CARRY_X       7.0
#define A_CARRY_Y       26.0
#define A_CARRY_W_H     -203.0
#define A_CARRY_W_C     10.0
#define A_CARRY_WAIST   0.0

#define A_ROCKL1_HATCH_X       25.5
#define A_ROCKL1_HATCH_Y       20.0 //21
#define A_ROCKL1_HATCH_W     -168.0
#define A_ROCKL1_HATCH_WAIST   0.0

#define A_ROCKL1_CARGO_X       7.0
#define A_ROCKL1_CARGO_Y       26.0
#define A_ROCKL1_CARGO_W     -10.0
#define A_ROCKL1_CARGO_WAIST   0.0

#define A_ROCKL2_HATCH_X       12.0
#define A_ROCKL2_HATCH_Y       48.0
#define A_ROCKL2_HATCH_W     -174.0
#define A_ROCKL2_HATCH_WAIST   0.0

#define A_ROCKL2_CARGO_X       6.0
#define A_ROCKL2_CARGO_Y       55.5
#define A_ROCKL2_CARGO_W      -1.0
#define A_ROCKL2_CARGO_WAIST   0.0

#define A_ROCKL3_HATCH_X       6.0
#define A_ROCKL3_HATCH_Y       75.0
#define A_ROCKL3_HATCH_W     -174.0
#define A_ROCKL3_HATCH_WAIST   0.0

#define A_ROCKL3_CARGO_X       4.0
#define A_ROCKL3_CARGO_Y       78.0
#define A_ROCKL3_CARGO_W_H     20.5
#define A_ROCKL3_CARGO_WAIST   0.0

#define A_CARGO_PICKUP_SUB_X       23.0
#define A_CARGO_PICKUP_SUB_Y       30.0 //25
#define A_CARGO_PICKUP_SUB_W     -34.0
#define A_CARGO_PICKUP_SUB_WAIST   0.0

#define A_CARGO_PICKUP_X       25.0
#define A_CARGO_PICKUP_Y       18.0
#define A_CARGO_PICKUP_W_H     -53.41
#define A_CARGO_PICKUP_WAIST   0.0

#define A_HATCH_PICKUP_SUB_X       22.0
#define A_HATCH_PICKUP_SUB_Y       31.0 //27
#define A_HATCH_PICKUP_SUB_W     -190.0
#define A_HATCH_PICKUP_SUB_WAIST   0.0

#define A_HATCH_PICKUP_X        25.5
#define A_HATCH_PICKUP_Y        18.0 //21
#define A_HATCH_PICKUP_W_H    -178.0 //-168
#define A_HATCH_PICKUP_WAIST    0.0

#define A_CARGO_HUMAN_X       9.0
#define A_CARGO_HUMAN_Y       41.0
#define A_CARGO_HUMAN_W_H     4.0
#define A_CARGO_HUMAN_WAIST   0.0

#define A_CSHIP_HATCH_X         25.5
#define A_CSHIP_HATCH_Y         21.0 //21
#define A_CSHIP_HATCH_W_H     -168.0
#define A_CSHIP_HATCH_WAIST     0.0

#define A_CSHIP_CARGO_X       12.0
#define A_CSHIP_CARGO_Y       48.0
#define A_CSHIP_CARGO_W_H     -33.0
#define A_CSHIP_CARGO_WAIST   0.0

#define A_FRAME_1_X       7.0
#define A_FRAME_1_Y       35.0
#define A_FRAME_1_W_H     -190.0
#define A_FRAME_1_WAIST   0.0

#define A_FRAME_2_X       7.0
#define A_FRAME_2_Y       35.0
#define A_FRAME_2_W_H     -248.0
#define A_FRAME_2_WAIST   0.0

#define A_FRAME_X       7.0
#define A_FRAME_Y       19.0
#define A_FRAME_W_H     -248.0
#define A_FRAME_WAIST   0.0

//Auto Scoring Positions
#define A_SC_START_POS_X       17.0
#define A_SC_START_POS_Y       23.0 //22.0
#define A_SC_START_POS_W_H     -174.0 //-182
#define A_SC_START_POS_WAIST   0.0

//Hatch Pickup Location
#define A_SC_PICKUP_X       23.17
#define A_SC_PICKUP_Y       19.5
#define A_SC_PICKUP_W_H     -195.5

//Rocket Low Level Height
#define A_SC_ROCKL1_HEIGHT  19.5

//Ready to place or pickup
#define A_SC_0_READY_DIST   14.0
#define A_SC_0_READY_Y      18.5
#define A_SC_0_READY_W      -178.0 //-195

#define A_SC_1_READY_DIST   12.0
//Y value is constant A_SC_ROCKL1_HEIGHT
#define A_SC_1_READY_W      -178.0 //-185

#define A_SC_2_READY_DIST   17.0
//Y value is constant A_ROCKL2_HATCH_Y
#define A_SC_2_READY_W      -174.0 //-181.5

#define A_SC_3_READY_DIST   20.0
//Y value is constant A_ROCKL3_HATCH_Y
#define A_SC_3_READY_W      -175.5

//Placing / Picking up
//7.5,19.5,-195.5
#define A_SC_GET_DIST     3.0 //7.5
//Y values same as above
//Wrist value same as above with the exception of high level
#define A_SC_3_GET_W       -173.5

//Return Values
#define A_SC_0_RETURN_X     15.0
#define A_SC_0_RETURN_Y     20.0
#define A_SC_0_RETURN_W     -179.0 //-187

#define A_SC_1_RETURN_X     15.0
#define A_SC_1_RETURN_W     -179.0 //-187

#define A_SC_2_RETURN_X     15.0
#define A_SC_2_RETURN_W     -174.5 //-181

#define A_SC_3_RETURN_X     6.0
#define A_SC_3_RETURN_W     -169.5 //-177



//Auto Ball Pickup
#define A_BP_START_X       18.0
#define A_BP_START_Y       33.5
#define A_BP_START_W     -60.0
#define A_BP_START_WAIST   0.0
//Delay height
#define A_BP_DELAY_W    -80.0
//Grab height
#define A_BP_GRAB       18.0

//Climbing
//Positions Above platform
#define A_C_START_X 25.0
#define A_CH_START_Y 44.0
#define A_CL_START_Y 30.0

//Wrist positions
#define A_C_WRIST -90.0
#define A_C_WRIST2 -93.0

//Position directly above the platform
#define A_C_START2_X 33.0
#define A_CH_START2_Y 30.0
#define A_CL_START2_Y 17.0

//Push down towards this position
#define A_C_X   33.0
#define A_C_Y      7.0

//Out of the way positions
#define A_CH_UP_X   33.0
#define A_CL_UP_X   20.0
#define A_C_UP_Y    27.0

#else 
//******************************************Practice bot*******************************************
#define A_CARRY_X       7.0
#define A_CARRY_Y       29.5
#define A_CARRY_W_H     -203.0
#define A_CARRY_W_C     -10.0
#define A_CARRY_WAIST   0.0

#define A_ROCKL1_HATCH_X       25.5
#define A_ROCKL1_HATCH_Y       21.0
#define A_ROCKL1_HATCH_W     -182.0
#define A_ROCKL1_HATCH_WAIST   0.0

#define A_ROCKL1_CARGO_X       7.0
#define A_ROCKL1_CARGO_Y       26.0
#define A_ROCKL1_CARGO_W     -10.0
#define A_ROCKL1_CARGO_WAIST   0.0

#define A_ROCKL2_HATCH_X       12.0
#define A_ROCKL2_HATCH_Y       48.0
#define A_ROCKL2_HATCH_W     -182.0
#define A_ROCKL2_HATCH_WAIST   0.0

#define A_ROCKL2_CARGO_X       6.0
#define A_ROCKL2_CARGO_Y       56.0
#define A_ROCKL2_CARGO_W      -1.0
#define A_ROCKL2_CARGO_WAIST   0.0

#define A_ROCKL3_HATCH_X       6.0
#define A_ROCKL3_HATCH_Y       75.0
#define A_ROCKL3_HATCH_W     -174.0
#define A_ROCKL3_HATCH_WAIST   0.0

#define A_ROCKL3_CARGO_X       4.0
#define A_ROCKL3_CARGO_Y       78.0
#define A_ROCKL3_CARGO_W_H     20.5
#define A_ROCKL3_CARGO_WAIST   0.0

#define A_CARGO_PICKUP_SUB_X       23.0
#define A_CARGO_PICKUP_SUB_Y       25.0
#define A_CARGO_PICKUP_SUB_W     -34.0
#define A_CARGO_PICKUP_SUB_WAIST   0.0

#define A_CARGO_PICKUP_X       25.0
#define A_CARGO_PICKUP_Y       16.0
#define A_CARGO_PICKUP_W_H     -53.41
#define A_CARGO_PICKUP_WAIST   0.0

#define A_HATCH_PICKUP_SUB_X       22.0
#define A_HATCH_PICKUP_SUB_Y       27.0
#define A_HATCH_PICKUP_SUB_W     -190.0
#define A_HATCH_PICKUP_SUB_WAIST   0.0

#define A_HATCH_PICKUP_X       25.0
#define A_HATCH_PICKUP_Y       19.5
#define A_HATCH_PICKUP_W_H     -174.0
#define A_HATCH_PICKUP_WAIST   0.0

#define A_CARGO_HUMAN_X       9.0
#define A_CARGO_HUMAN_Y       41.0
#define A_CARGO_HUMAN_W_H     4.0
#define A_CARGO_HUMAN_WAIST   0.0

#define A_CSHIP_HATCH_X       25.5
#define A_CSHIP_HATCH_Y       19.0
#define A_CSHIP_HATCH_W_H     -182.0
#define A_CSHIP_HATCH_WAIST   0.0

#define A_CSHIP_CARGO_X       18.0
#define A_CSHIP_CARGO_Y       48.0
#define A_CSHIP_CARGO_W_H     -33.0
#define A_CSHIP_CARGO_WAIST   0.0

#define A_FRAME_1_X       7.0
#define A_FRAME_1_Y       35.0
#define A_FRAME_1_W_H     -190.0
#define A_FRAME_1_WAIST   0.0

#define A_FRAME_2_X       7.0
#define A_FRAME_2_Y       35.0
#define A_FRAME_2_W_H     -248.0
#define A_FRAME_2_WAIST   0.0

#define A_FRAME_X       7.0
#define A_FRAME_Y       19.0
#define A_FRAME_W_H     -248.0
#define A_FRAME_WAIST   0.0

//Auto Scoring Positions
#define A_SC_START_POS_X       17.0
#define A_SC_START_POS_Y       22.0
#define A_SC_START_POS_W_H     -182.0
#define A_SC_START_POS_WAIST   0.0

//Hatch Pickup Location
#define A_SC_PICKUP_X       23.17
#define A_SC_PICKUP_Y       19.5
#define A_SC_PICKUP_W_H     -195.5

//Rocket Low Level Height
#define A_SC_ROCKL1_HEIGHT  19.5

//Ready to place or pickup
#define A_SC_0_READY_DIST   14.0
#define A_SC_0_READY_Y      19.5
#define A_SC_0_READY_W      -195.0

#define A_SC_1_READY_DIST   12.0
//Y value is constant A_SC_ROCKL1_HEIGHT
#define A_SC_1_READY_W      -185.0

#define A_SC_2_READY_DIST   14.0
//Y value is constant A_ROCKL2_HATCH_Y
#define A_SC_2_READY_W      -181.5

#define A_SC_3_READY_DIST   16.0
//Y value is constant A_ROCKL3_HATCH_Y
#define A_SC_3_READY_W      -175.5

//Placing / Picking up
//7.5,19.5,-195.5
#define A_SC_GET_DIST     7.5
//Y values same as above
//Wrist value same as above with the exception of high level
#define A_SC_3_GET_W       -173.5

//Return Values
#define A_SC_0_RETURN_X     20.0
#define A_SC_0_RETURN_Y     20.0
#define A_SC_0_RETURN_W     -187.0

#define A_SC_1_RETURN_X     15.0
#define A_SC_1_RETURN_W     -187.0

#define A_SC_2_RETURN_X     15.0
#define A_SC_2_RETURN_W     -181.5

#define A_SC_3_RETURN_X     6.0
#define A_SC_3_RETURN_W     -177.5

//Auto Ball Pickup
#define A_BP_START_X       18.0
#define A_BP_START_Y       27.5
#define A_BP_START_W     -60.0
#define A_BP_START_WAIST   0.0
//Delay height
#define A_BP_DELAY_W    -80.0
//Grab height
#define A_BP_GRAB       18.0

//Climbing
//Positions Above platform
#define A_C_START_X 25.0
#define A_CH_START_Y 44.0
#define A_CL_START_Y 30.0

//Wrist positions
#define A_C_WRIST -90.0
#define A_C_WRIST2 -93.0

//Position directly above the platform
#define A_C_START2_X 33.0
#define A_CH_START2_Y 30.0
#define A_CL_START2_Y 17.0

//Push down towards this position
#define A_C_X   33.0
#define A_C_Y      10.0

//Out of the way positions
#define A_CH_UP_X   33.0
#define A_CL_UP_X   20.0
#define A_C_UP_Y    27.0

#endif //If flight



#endif