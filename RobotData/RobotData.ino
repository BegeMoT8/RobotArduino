//собрать данные с гироскопа MPU6050, компаса HMC5883L, дальномера HC-SR04

#include "Wire.h"
#include <Vector.h>

// пины дальномера
#define HC_TRIG 3 //выход
#define HC_ECHO 2 //вход
// адрес датчика MPU6050
const int MPU_addr = 0x68; 

void sensorInit()
{
	// HC-SR 04
	pinMode(HC_TRIG, OUTPUT);
	pinMode(HC_ECHO, INPUT);
	// MPU6050
	Wire.begin();
	Wire.beginTransmission(MPU_addr);
	Wire.write(0x6B);  // PWR_MGMT_1 register
	Wire.write(0);     // set to zero (wakes up the MPU-6050)
	Wire.endTransmission(true);
}



void setup() {
	Serial.begin(115200);    
}


void loop() {

}

float getSonarvalue()
{
	// минимальное время опроса сонара (не рекомендуется опрашивать чаще 30 мс)
	const uint8_t callDelay = 50;

	static uint32_t lastCall = 0;
	while (millis() - lastCall < callDelay);
	lastCall = millis();

	// импульс 10 мкс
	digitalWrite(HC_TRIG, HIGH);
	delayMicroseconds(10);
	digitalWrite(HC_TRIG, LOW);

	// измеряем время ответного импульса (микросекунды) timeout = 0.006 с
	// сигнал проходит за это время 2 м в одну сторону
	uint32_t timeImpuls = pulseIn(HC_ECHO, HIGH, 6000);
	return (timeImpuls == 0 ? -1 : timeImpuls * 0.01715);
}

Vector<int>& getGyro()
{
	// [accX, accY, accZ, temp, gyrX, gyrY, gyrZ]
	// acc - ускорение, gyr - угловая скорость, temp - температура (raw)
	static Vector<int> data;
	// отправка данных на	
	Wire.beginTransmission(MPU_addr);
	//регистр с которого начинаем чтение данных (данные акселерометра по оси X)
	Wire.write(0x3B);
	Wire.endTransmission(false);
	// Запрос данных со всех регистров
	Wire.requestFrom(MPU_addr, 14, true); 
	
	//чтение парами байтов (каждое число 16 бит)
	for (byte i = 0; i < 7; i++) {
		data.push_back( Wire.read() << 8 | Wire.read() );
	}
	return data;
}