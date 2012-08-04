#ifndef _VERSION_H_
#define _VERSION_H_

#if TRACKER_BOARD_REVISION == 1
#define LSM303DLM
#elif TRACKER_BOARD_REVISION == 2
#define LSM303DLHC
#define LTC3554
#elif TRACKER_BOARD_REVISION == 3
#define LSM303DLHC
#define LTC3553
#else
#error Need to define a TRACKER_BOARD_REVISION
#endif

#endif /* _VERSION_H_ */
