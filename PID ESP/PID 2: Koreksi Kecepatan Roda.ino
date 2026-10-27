// Mixing gerak dasar (Vx = geser kanan/kiri, Vy = maju/mundur, Omega = koreksiYaw)
float targetSpeed[4];
targetSpeed[0] = Vy + Vx + koreksiYaw; // Motor Depan-Kiri
targetSpeed[1] = Vy - Vx - koreksiYaw; // Motor Depan-Kanan
targetSpeed[2] = Vy - Vx + koreksiYaw; // Motor Belakang-Kiri
targetSpeed[3] = Vy + Vx - koreksiYaw; // Motor Belakang-Kanan

// Inisialisasi PID per roda
PIDController pidRoda[4] = {
  {1.2, 0.2, 0.01, 0, 0, 0, 255},
  {1.2, 0.2, 0.01, 0, 0, 0, 255},
  {1.2, 0.2, 0.01, 0, 0, 0, 255},
  {1.2, 0.2, 0.01, 0, 0, 0, 255}
};

// Hitung output PWM akhir untuk masing-masing roda
int8_t pwmMotor[4];
for (int i = 0; i < 4; i++) {
  float speedNyata = hitungSpeedEncoder(i); // Kecepatan terukur (RPM/Tick)
  pwmMotor[i] = (int8_t) pidRoda[i].hitung(targetSpeed[i], speedNyata, dt);
}