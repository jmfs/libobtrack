#ifndef _OBTRACK_H
#define _OBTRACK_H

#include "CamShiftTracker.h"
#include "FASTrack.h"
#include "RotatedRect.h"
#include "Rect.h"

#ifdef _OBTRACK_USE_KINECT
#include "Kinect.h"
#endif

#ifdef _OBTRACK_USE_TLD
#include "TLDTracker.h"
#endif

#endif