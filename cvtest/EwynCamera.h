#ifndef __EWYN_CAMERA_H
#define __EWYN_CAMERA_H

class EWynCamera {
public:
	EWynCamera();

private:
	// Thresholding limits.
	int	lowHueThreshold;
	int	highHueThreshold;
	int lowSaturationThreshold;
	int highSaturationThreshold;
	int lowValueThreshold;
	int highValueThreshold;
};
#endif