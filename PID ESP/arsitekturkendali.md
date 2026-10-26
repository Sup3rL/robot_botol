[Remote: Maju/Mundur/Geser] ──> Target Kecepatan & Sudut
                                      │
[Sensor Kompas] ──────────────> [PID Heading] ──> Nilai Koreksi Belok (Yaw)
                                      │
                              Kinematika 4 Roda (Mixing)
                                      │
[Sensor Encoder] ─────────────> [PID Kecepatan x4 Roda] ──> PWM Motor 1, 2, 3, 4
                                      │
                                  [ESP-NOW] (Delay ~1 ms)
                                      │
                                    ESP 1 ──> Motor Driver & Gripper



Urutan Penalaan Parameter (Tuning)Matikan PID Roda terlebih dahulu: Uji respons motor manual dari remote ke ESP 1 untuk memastikan arah putaran motor sudah sinkron.Tuning PID Kompas: Tempatkan robot di lantai, jalankan maju lurus. Naikkan $K_p$ kompas sampai robot bisa mempertahankan orientasi lurus saat didorong atau oleng secara manual. Jika robot bergoyang kanan-kiri (osilasi), naikkan $K_d$.Tuning PID Roda: Jika roda selip atau salah satu motor lebih lemah, naikkan $K_i$ pada PID encoder roda agar RPM antar roda seimbang.

link percakapan Gemini, takut bingung cara pakainya

https://share.gemini.google/P4gEbaL8Mqcq 