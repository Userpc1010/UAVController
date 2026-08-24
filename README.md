# UAV Controller

Система управления БПЛА (V-Tail) с GPS, GSM, IMU и PID-регуляторами.

<img width="960" height="720" alt="61d7fa23bfc1c4a76018dbf13eb0adb6" src="https://github.com/user-attachments/assets/c09a4a2b-9c80-4c73-911e-e88e2e877b88" />

<img width="3264" height="2448" alt="IMG_20191128_152307_679" src="https://github.com/user-attachments/assets/13c097c1-47f2-4ea0-b718-a8cc1b0a589c" />

<img width="3264" height="2448" alt="IMG_20191128_152352_664" src="https://github.com/user-attachments/assets/667389ba-a75b-4b20-95e2-6bb5226830b2" />


## Возможности

- 🛰️ **GPS** (u-blox M8N, UBX NAV-PVT) — навигация, маршруты, LOS guidance
- 📡 **GSM** — отправка координат по SMS при звонке
- 🧭 **IMU** (BNO080) — ориентация (кватернионы, pitch, roll, yaw)
- 🌡️ **MS5611** — барометр (альтитуда)
- 📏 **LiDAR TFmini** — альтитуда с компенсацией наклона
- 🎮 **PID-регуляторы** — altitude, bank, pitch, roll, yaw
- 🧮 **Kalman Filter** — fusion GPS + IMU
- 💾 **EEPROM** — хранение PID, триммеров, лимитов
- 🖥️ **OpenGL** — визуализация дрона
- 🔄 **Протоколы** — PID, LIM, TRIM, MOVE, ROUT

## Навигация

- Полёт к точке (MOVE)
- Полёт по маршруту (ROUT)
- Циркуляризация (circle)
- LOS guidance law
- Kalman Filter (GPS + IMU fusion)

## Структура

- `UAV_Core/` — основная программа
  - `gps.cpp/h` — GPS приёмник и навигация
  - `gsm.cpp/h` — GSM модем
  - `i2c_ports.cpp/h` — I2C датчики и PWM
  - `BNO080.cpp/h` — IMU
  - `ms5611.cpp/h` — барометр
  - `lidar_tfmini.cpp/h` — лидар
  - `pca9685.cpp/h` — PWM контроллер
  - `myserver.cpp/h` — TCP сервер
  - `GpsImuFusionKalmon.cpp/h` — фильтр Калмана
- `Voxels/` — визуализация (OpenGL)

## Протоколы UART

PID1 + struct PID — сохранение PID
TRIM1 + struct Trimm — сохранение триммеров
LIM1 + struct Limit — сохранение лимитов
MOVE + struct Move — точка полёта
ROUT + struct Rout — маршрут
TELE + struct Telemetry — телеметрия

## Требования

    Qt 5.12+

    WiringPi

    OpenGL

## Аппаратная часть

    Raspberry Pi (или аналог)

    BNO080 (IMU)

    u-blox M8N (GPS)

    GSM модем

    MS5611 (барометр)

    TFmini (лидар)

    PCA9685 (PWM)

# Описание

https://rc.lyxstv.ru/forum/topic/2843/

https://temofeev.com/info/articles/samodelnyy-avtopilot-na-odnopltanom-kompyutere-sbc-tinker-board-i-arduino-due/

## Сборка

```bash
mkdir build && cd build
qmake ../UAV.pro
make -j$(nproc)
```

## Описание для GitHub

UAV control system with GPS, GSM, IMU, PID controllers and Kalman filter




