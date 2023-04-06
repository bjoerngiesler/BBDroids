#if !defined(IMUFILTER_H)
#define IMUFILTER_H

#include <vector>

class IMUFilter {
public:
  static IMUFilter imu;
  bool begin();
  bool available() { return available_; }
  void update();
  void getFilteredEulerAngles(float& roll, float& pitch, float& yaw);
  void getRawEulerAngles(float& roll, float& pitch, float& yaw);

protected:
  IMUFilter();

  bool available_;
  bool initialized_;
  unsigned int lastMillis_;
  float r_, p_, y_;
  std::vector<float> rolls_, pitches_, yaws_;
};

#endif // IMUFILTER_H