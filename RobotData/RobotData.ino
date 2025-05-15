//������� ������ � ��������� MPU6050, ������� HMC5883L, ���������� HC-SR04

#include "Wire.h"
#include <Vector.h>

// ���� ����������
#define HC_TRIG 3 //�����
#define HC_ECHO 2 //����
// ����� ������� MPU6050
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
	// ����������� ����� ������ ������ (�� ������������� ���������� ���� 30 ��)
	const uint8_t callDelay = 50;

	static uint32_t lastCall = 0;
	while (millis() - lastCall < callDelay);
	lastCall = millis();

	// ������� 10 ���
	digitalWrite(HC_TRIG, HIGH);
	delayMicroseconds(10);
	digitalWrite(HC_TRIG, LOW);

	// �������� ����� ��������� �������� (������������) timeout = 0.006 �
	// ������ �������� �� ��� ����� 2 � � ���� �������
	uint32_t timeImpuls = pulseIn(HC_ECHO, HIGH, 6000);
	return (timeImpuls == 0 ? -1 : timeImpuls * 0.01715);
}

Vector<int>& getGyro()
{
	// [accX, accY, accZ, temp, gyrX, gyrY, gyrZ]
	// acc - ���������, gyr - ������� ��������, temp - ����������� (raw)
	static Vector<int> data;
	// �������� ������ ��	
	Wire.beginTransmission(MPU_addr);
	//������� � �������� �������� ������ ������ (������ ������������� �� ��� X)
	Wire.write(0x3B);
	Wire.endTransmission(false);
	// ������ ������ �� ���� ���������
	Wire.requestFrom(MPU_addr, 14, true); 
	
	//������ ������ ������ (������ ����� 16 ���)
	for (byte i = 0; i < 7; i++) {
		data.push_back( Wire.read() << 8 | Wire.read() );
	}
	return data;
}