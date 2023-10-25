#include <BBLowPassFilter.h>
#include <Arduino.h>

bb::LowPassFilter::LowPassFilter(float cutoff, float sampleFreq, bool adaptive) {
	cutoff_ = cutoff;
	sampleFreq_ = sampleFreq;
    adapt_ = adaptive;

    needsRecalc_ = true;

    dt_ = 1.0/sampleFreq_;
    tn1_ = -dt_;
    for(int k = 0; k < 3; k++){
        x_[k] = 0;
        y_[k] = 0;        
    }
    computeCoefficients();
}

void bb::LowPassFilter::setCutoff(float cutoff) {
	cutoff_ = cutoff;
	needsRecalc_ = true;
}

void bb::LowPassFilter::setSampleFrequency(float sampleFreq) {
	sampleFreq_ = sampleFreq;
    dt_ = 1.0/sampleFreq_;
    tn1_ = -dt_;
	needsRecalc_ = true;
}

void bb::LowPassFilter::setAdaptive(bool adaptive) {
	adapt_ = adaptive;
	needsRecalc_ = true;
}


void bb::LowPassFilter::computeCoefficients() {
	omega0_ = 6.28318530718*cutoff_;

	if(adapt_){
        float t = micros()/1.0e6;
        dt_ = t - tn1_;
        tn1_ = t;
	}
      
	float alpha = omega0_*dt_;
    float alphaSq = alpha*alpha;
    float beta[] = {1, sqrt(2), 1};
    float D = alphaSq*beta[0] + 2*alpha*beta[1] + 4*beta[2];
    b_[0] = alphaSq/D;
    b_[1] = 2*b_[0];
    b_[2] = b_[0];
    a_[0] = -(2*alphaSq*beta[0] - 8*beta[2])/D;
    a_[1] = -(beta[0]*alphaSq - 2*beta[1]*alpha + 4*beta[2])/D;      
}

float bb::LowPassFilter::filter(float xn) {
	// Provide me with the current raw value: x
	// I will give you the current filtered value: y
	if(adapt_ || needsRecalc_){
		computeCoefficients(); // Update coefficients if necessary   
		needsRecalc_ = false;   
	}
	y_[0] = 0;
	x_[0] = xn;
	// Compute the filtered values
	for(int k = 0; k < 2; k++){
		y_[0] += a_[k]*y_[k+1] + b_[k]*x_[k];
	}
	y_[0] += b_[2]*x_[2];

	// Save the historical values
	for(int k = 2; k > 0; k--){
		y_[k] = y_[k-1];
		x_[k] = x_[k-1];
	}

	// Return the filtered value    
	return y_[0];
}
