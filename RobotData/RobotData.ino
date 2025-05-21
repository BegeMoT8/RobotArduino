#include <Vector.h>
#include "Wire.h"

// пины HC-SR04
#define HC_TRIG 3 //выход
#define HC_ECHO 2 //вход

// MPU6050
const int MPU_addr = 0x68;
const int MPU_start_Reg = 0x3B;
const int sensAngular = 131; // чувствительность угловая скорость +/- 250 град / с
const int sensAccel = 16384.0; // чувствительность ускорение +/- 2g 

// HMC5883L
const int HMC5883L_addr = 0x1E;

// L298N
const int ENA = 10, IN1 = 9, IN2 = 8;//мотор A
const int ENB = 5, IN3 = 7, IN4 = 6;// Мотор B
const int ENC = 11, IN5 = 12, IN6 = 13;// Мотор C

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
	// настройка диапозона ускорений
	Wire.write(0x1C);                   //обращаемся к регистру ACCEL_CONFIG (1C hex)
	Wire.write(0x00);                   //Установить биты регистра как 00000000 (диапазон полной шкалы +/- 2g)
	// настройка диапозона угловой скорости
	Wire.write(0x1B);                   // Обращаемся к регистру GYRO_CONFIG (1B hex)
	Wire.write(0x00);                   // Установить биты регистра как 00000000 (полная шкала  +/- 250 град / с)
	Wire.endTransmission(true);
	// HMC5883L
	Wire.beginTransmission(HMC5883L_addr);
	Wire.write(0x00); // выбираем регистр управления CRA (00)
	Wire.write(0x70); // записываем в него 0x70 [усреднение по 8 точкам, 15 Гц, нормальные измерения]
	Wire.write(0xA0); // записываем в регистр CRB (01) 0xA0 [чувствительность = 5]
	Wire.write(0x00); // записываем в регистр Mode (02) 0x00 [бесконечный режим измерения]
	Wire.endTransmission();
	// L298N
	pinMode(ENA, OUTPUT);
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(ENB, OUTPUT);
	pinMode(IN3, OUTPUT);
	pinMode(IN4, OUTPUT);
	pinMode(ENC, OUTPUT);
	pinMode(IN5, OUTPUT);
	pinMode(IN6, OUTPUT);
}

//расстояние с дальномера в см
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
//получает данные с гироскопа в порядке 
//[accX, accY, accZ, temp, gyrX, gyrY, gyrZ]
void getGyro(Vector<float>& data)
{
	//хранит 16 битное значение с гироскопа
	Vector<int> temp;
	Wire.beginTransmission(MPU_addr);
	//регистр с которого начинаем чтение данных (данные акселерометра по оси X)
	Wire.write(MPU_start_Reg);
	Wire.endTransmission(false);

	// Запрос данных со всех регистров
	if (Wire.requestFrom(MPU_addr, 14, true) != 14) {
		//число полученных байт не равно количеству запрошенных
		for (int i = 0; i < 7; i++) temp.push_back(0);
	}
	else {
		//чтение парами байтов (каждое число 16 бит)
		for (byte i = 0; i < 7; i++) {
			temp.push_back(Wire.read() << 8 | Wire.read());
		}
	}
	//проверка доступных ячеек
	if (data.size() == 0 )	{
		for (int i = 0; i < 7; i++) data.push_back(0.0f);
	}

	//перевод в СИ [м/с^2, м/с^2, м/с^2, Цельсий, рад, рад, рад]
	for (int i = 0; i < temp.size(); i++){
		if (i < 3) {
			data[i] = ((temp[i] / sensAccel) * 9.81);
		}
		else if (i == 3) {
			data[i] = ((temp[i] / 340) + 36.53);
		}
		else {
			data[i] = ((temp[i] / sensAngular) * PI/180);
		}
	}
}

// угол на север (рад)
float getCompas()
{
	double x, y;
	Wire.beginTransmission(HMC5883L_addr);
	Wire.write(0x03); // переходим к регистру 0x03
	Wire.endTransmission(false);
	// запрашиваем 6 байтов
	Wire.requestFrom(HMC5883L_addr, 4); 
	while (Wire.available()) {
		// объединяем в двухбайтовое число
		x = Wire.read() << 8 | Wire.read();
		y = Wire.read() << 8 | Wire.read();
	}
	float azimut = atan2(y, x); // высчитываем направление
	//нахождение в диапазоне от 0 до 2π радиан
	if (azimut < 0) azimut += 2 * PI;
	if (azimut > 2 * PI) azimut -= 2 * PI;
	return azimut;
}

// rot. вращение на величину rot/ положительное - почасовой/ отрицательное - против часовой
// deg - направление движение; 0 - прямо, 90 - направо, -90 - налево.
void move(int rot, int deg, int speed)
{
	// параметры робота
	const static float massa = 2; // масса робота кг
	const static float J = 0.5; // момент инерции робота
	const static float R = 0.2; // расстояние от центра колеса до центра платформы, м
	const static float maxSpeed = 0.5; // макс скорость м/с
	const static float scaleFactor = 255 / maxSpeed;

	// скорости колес v1, v2, v3
	// massa - масса робота 
	// Vx, Vy - желаемые скорости по осям координат
	// J - момент инерции робота 
	// rot - угловая скорость мобильной платформы (рад/с)

	// Преобразуем градусы в радианы
	deg = deg * PI / 180.0;

	//скорости Vx и Vy
	// общий знаменталель
	float denominator = sqrt(pow(tan(deg), 2) + 1);

	float Vx = speed * tan(deg) * (1 / denominator);
	float Vy = speed * (1 / denominator);

	// рассчет скоростей на каждом колесе
	float v1 = (massa / 3) * (sqrt(3) * Vy - Vx + J * R * rot);
	float v2 = -(massa / 3) * (sqrt(3) * Vy + Vx - J * R * rot);
	float v3 = (massa / 3) * (2 * Vx + J * R * rot);

	// Масштабирование до диапазона PWM, сохраняя знак
	v1 = v1 * scaleFactor;
	v2 = v2 * scaleFactor;
	v3 = v3 * scaleFactor;

	setMotor(ENA, IN1, IN2, v1);
	setMotor(ENB, IN3, IN4, v2);
	setMotor(ENC, IN5, IN6, v3);
}

// Функция для установки скорости и направления мотора
void setMotor(int enablePin, int in1, int in2, float velocity) {
	int pwmValue = abs(velocity); // Абсолютное значение скорости для PWM

	// Ограничиваем PWM значением 0-255
	pwmValue = constrain(pwmValue, 0, 255);

	if (velocity > 0) {
		digitalWrite(in1, HIGH);
		digitalWrite(in2, LOW);
	}
	else if (velocity < 0) {
		digitalWrite(in1, LOW);
		digitalWrite(in2, HIGH);
	}
	else {
		// Остановка мотора
		digitalWrite(in1, LOW);
		digitalWrite(in2, LOW);
	}
	analogWrite(enablePin, pwmValue);
}

bool checkHit()
{
	float dist = getSonarvalue();
	return (dist < 4 ? true : false);
}
