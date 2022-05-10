
class RemoteDisplay {
  public:
  RemoteDisplay();

  void update();
  void setConnected(bool conn);

  protected:
  bool connected_;
  bool left_led_state_, right_led_state_;
  unsigned long last_millis_;
};
