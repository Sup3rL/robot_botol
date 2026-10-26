// Normalisasi error ke rentang -180 sampai +180 derajat
float hitungErrorSudut(float target, float current) {
  float error = target - current;
  while (error > 180.0)  error -= 360.0;
  while (error < -180.0) error += 360.0;
  return error;
}

//Implementasi di loop pembacaan ESP 2:

PIDController pidKompas = {2.5, 0.05, 0.12, 0, 0, -100, 100}; // Nilai tuning awal

float sudutKompasSekarang = bacaKompas(); 
float errorSudut = hitungErrorSudut(targetSudutRemote, sudutKompasSekarang);

// Koreksi yaw dari PID
float koreksiYaw = pidKompas.hitung(errorSudut, 0, dt);