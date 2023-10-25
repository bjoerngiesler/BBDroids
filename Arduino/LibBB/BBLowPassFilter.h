#if !defined(BBFILTER_H)
#define BBFILTER_H

namespace bb {

// Taken from the tutorial and example code of the EXCELLENT Curio Res (https://www.youtube.com/@curiores111, 
// https://github.com/curiores/ArduinoTutorials/blob/main/BasicFilters/ArduinoImplementations/LowPass/LowPass2.0/LowPass2.0.ino)
// This realizes a 2nd order Butterworth low pass filter with configurable cutoff frequency.
class LowPassFilter {
public:
	LowPassFilter(float cutoff, float sampleFreq, bool adaptive);

	float cutoff() { return cutoff_; }
	void setCutoff(float cutoff);

	float sampleFrequency() { return sampleFreq_; }
	void setSampleFrequency(float sampleFreq);

	bool adaptive() { return adapt_; }
	void setAdaptive(bool adaptive);

	float filter(float xn);
protected:
	void computeBase();
	void computeCoefficients();
	float a_[2], b_[3];
	float omega0_;
	float dt_;
	bool adapt_;
	float tn1_ = 0;
	float x_[3], y_[3];

	float cutoff_, sampleFreq_;

	bool needsRecalc_;
};

}

#endif // BBFILTER_H