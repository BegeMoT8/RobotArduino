//������� ������ � ��������� MPU6050, ������� HMC5883L, ���������� HC-SR04


// ���� ����������
#define HC_TRIG 3 //�����
#define HC_ECHO 2 //����
void sensorInit()
{
	pinMode(HC_TRIG, OUTPUT);
	pinMode(HC_ECHO, INPUT);
}

void setup() {
	Serial.begin(115200);       

}
float getSonarvalue()
{
	//�� ������������� ���������� ���� 30 ��
	static uint32_t callDelay = 0;
	while (callDelay - millis() < 50);
	callDelay = millis();

	// ������� 10 ���
	digitalWrite(HC_TRIG, HIGH);
	delayMicroseconds(10);
	digitalWrite(HC_TRIG, LOW);

	// �������� ����� ��������� �������� (������������)
	uint32_t timeImpuls = pulseIn(HC_ECHO, HIGH);

	// ��������� ����������� � ����������
	static float distFilt = 0;
	float dist = 343 * (timeImpuls * 10^(-6)) / 2;   
	distFilt += (dist - distFilt) * 0.2;  // 0.2 - ���� ����������

	return distFilt;
}

void loop() {

}


void setup()
{
	

}


void loop()
{


}
