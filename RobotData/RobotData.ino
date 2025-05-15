//собрать данные с гироскопа MPU6050, компаса HMC5883L, дальномера HC-SR04


// пины дальномера
#define HC_TRIG 3 //выход
#define HC_ECHO 2 //вход
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
	//Не рекомендуется опрашивать чаще 30 мс
	static uint32_t callDelay = 0;
	while (callDelay - millis() < 50);
	callDelay = millis();

	// импульс 10 мкс
	digitalWrite(HC_TRIG, HIGH);
	delayMicroseconds(10);
	digitalWrite(HC_TRIG, LOW);

	// измеряем время ответного импульса (микросекунды)
	uint32_t timeImpuls = pulseIn(HC_ECHO, HIGH);

	// Получение рассчтояния и фильтрация
	static float distFilt = 0;
	float dist = 343 * (timeImpuls * 10^(-6)) / 2;   
	distFilt += (dist - distFilt) * 0.2;  // 0.2 - коэф фильтрации

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
