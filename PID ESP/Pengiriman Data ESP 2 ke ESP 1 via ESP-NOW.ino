typedef struct PacketData {
  int8_t speed_m1;      // PWM Motor 1 (-100 s.d 100 atau 0 s.d 255)
  int8_t speed_m2;      // PWM Motor 2
  int8_t speed_m3;      // PWM Motor 3
  int8_t speed_m4;      // PWM Motor 4
  uint8_t gripper_cmd;  // 0: Lepas, 1: Jepit, atau sudut servo
} PacketData;

PacketData dataKirim;

//kirim dari esp 2

dataKirim.speed_m1 = pwmMotor[0];
dataKirim.speed_m2 = pwmMotor[1];
dataKirim.speed_m3 = pwmMotor[2];
dataKirim.speed_m4 = pwmMotor[3];
dataKirim.gripper_cmd = statusGripperRemote;

esp_now_send(macAddressESP1, (uint8_t *) &dataKirim, sizeof(dataKirim));

//terima di esp 1

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  PacketData dataTerima;
  memcpy(&dataTerima, incomingData, sizeof(dataTerima));

  // Terapkan PWM langsung ke motor driver
  setMotorPWM(1, dataTerima.speed_m1);
  setMotorPWM(2, dataTerima.speed_m2);
  setMotorPWM(3, dataTerima.speed_m3);
  setMotorPWM(4, dataTerima.speed_m4);

  // Kendali gripper
  kendaliGripper(dataTerima.gripper_cmd);
}