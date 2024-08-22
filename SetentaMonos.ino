/*
 Name:		SetentaMonos.ino
 Created:	20/08/2024 16:40:34
 Author:	Ofel
*/

#include <TMCStepper.h>
#include <TMCStepper_UTILITY.h>
#include <ArduinoSTL.h>
#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

constexpr auto L1 = 100.0; // Longitud del primer eslabón (hombro) en mm
constexpr auto L2 = 100.0; // Longitud del segundo eslabón (codo) en mm

// Configuración del motor del hombro (NEMA17 con TMC2209)
constexpr auto STEP_PIN = 6;
constexpr auto DIR_PIN = 3;
constexpr auto ENABLE_PIN = 51;
constexpr auto R_SENSE = 0.11f;      // R_SENSE para cálculo de corriente
constexpr auto DRIVER_ADDRESS = 0b00;       // Dirección del driver TMC2209 según MS1 y MS2
#define SERIAL_PORT Serial3

// Configuración del motor del codo (24BYJ48 con ULN2003)
constexpr auto HALFSTEP = 8;
constexpr auto motorPin1 = 8;     // IN1 on ULN2003 ==> Blue   on 28BYJ-48
constexpr auto motorPin2 = 9;     // IN2 on ULN2004 ==> Pink   on 28BYJ-48
constexpr auto motorPin3 = 10;    // IN3 on ULN2003 ==> Yellow on 28BYJ-48
constexpr auto motorPin4 = 11;    // IN4 on ULN2003 ==> Orange on 28BYJ-48

constexpr auto LEVA_HOMBRO_PIN = 5;
constexpr auto LEVA_CODO_PIN = 4;

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
AccelStepper hombro(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
AccelStepper codo(HALFSTEP, motorPin1, motorPin3, motorPin2, motorPin4);
MultiStepper motores;

struct Angles {
	double theta1;
	double theta2;
};

struct Coordenadas {
	double x;
	double y;
};

long posiciones[2]; // Array para almacenar las posiciones objetivo
Coordenadas calcularCoordenadasDesdeCentro(String notacion) {
	// Dimensiones del tablero
	constexpr double ladoTablero = 323.25;
	constexpr double tamanoCasilla = ladoTablero / 8.0;

	// Coordenadas del centro del tablero
	constexpr double centroTablero = ladoTablero / 2.0;

	// Convertir notación de ajedrez a índices de fila y columna
	int columna = notacion[0] - 'a'; // Columna [a-h] -> [0-7]
	int fila = notacion[1] - '1';    // Fila [1-8] -> [0-7]

	// Calcular coordenadas desde el centro del tablero
	double x = (columna + 0.5) * tamanoCasilla - centroTablero;
	double y = (fila + 0.5) * tamanoCasilla - centroTablero;

	Coordenadas coord = { x, y };
	return coord;
}

void realizarHoming() {
	// Homing para el hombro
	bool estadoLeva = digitalRead(LEVA_HOMBRO_PIN);
	if (estadoLeva == LOW) {
		// Mover el motor hasta que se active el fin de carrera
		hombro.setSpeed(-500);
		while (digitalRead(LEVA_HOMBRO_PIN) == LOW) {
			hombro.runSpeed();
		}
	}
	else {
		hombro.setSpeed(500);
		while (digitalRead(LEVA_HOMBRO_PIN) == HIGH) {
			hombro.runSpeed();
		}
	}
	hombro.setCurrentPosition(0); // Establece la posición actual como cero

	// Homing para el codo
	estadoLeva = digitalRead(LEVA_CODO_PIN);
	if (estadoLeva == LOW) {
		// Mover el motor hasta que se active el fin de carrera
		codo.setSpeed(500);
		while (digitalRead(LEVA_CODO_PIN) == LOW) {
			codo.runSpeed();
		}
	}
	else {
		codo.setSpeed(-500);
		while (digitalRead(LEVA_CODO_PIN) == HIGH) {
			codo.runSpeed();
		}
	}
	codo.setCurrentPosition(0); // Establece la posición actual como cero
	posiciones[0] = 2270;
	posiciones[1] = -770;
	// Mover los motores a la posición deseada
	motores.moveTo(posiciones);
	motores.runSpeedToPosition();
	// Establece la posición actual como cero
	codo.setCurrentPosition(0);
	hombro.setCurrentPosition(0);

}

void moverAPosicion(String comando) {
	int columna = comando[0] - 'a'; // Columna [a-h]
	int fila = comando[1] - '1';    // Fila [1-8]
	Serial.print("Moviendo a la posicion "); Serial.print(comando); Serial.println("...");
	// Convertir notación de ajedrez a coordenadas en mm desde el centro del tablero
	Coordenadas coord = calcularCoordenadasDesdeCentro(comando);
	double x = coord.x;
	double y = coord.y;
	Serial.print("Coordenadas: ("); Serial.print(x); Serial.print(", "); Serial.print(y); Serial.println(")");

	// Convertir coordenadas a ángulos (cinemática inversa)
	Angles angulos = calcularAngulos(x, y);
	Serial.print("Angulos: ("); Serial.print(angulos.theta1); Serial.print(", "); Serial.print(angulos.theta2); Serial.println(")");

	// Convertir ángulos a posiciones de motor (en pasos)
	posiciones[0] = calcularPasosHombro(angulos.theta1);
	posiciones[1] = calcularPasosCodo(angulos.theta2);
	Serial.print("Pasos: ("); Serial.print(posiciones[0]); Serial.print(", "); Serial.print(posiciones[1]); Serial.println(")");

	// Mover los motores a la posición deseada
	motores.moveTo(posiciones);
	motores.runSpeedToPosition();
}

long calcularPasosHombro(double theta1) {
	double pasosPorRev = 8120 / 360;
	long pasos = theta1 * pasosPorRev;
	return pasos;
}

Angles calcularAngulos(double x, double y) {
	Angles angulos1, angulos2;
	double D = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);

	// Calcular los dos posibles ángulos del codo
	angulos1.theta2 = atan2(sqrt(1 - D * D), D); // Codo arriba
	angulos2.theta2 = atan2(-sqrt(1 - D * D), D); // Codo abajo

	// Calcular los ángulos del hombro correspondientes
	angulos1.theta1 = atan2(y, x) - atan2(L2 * sin(angulos1.theta2), L1 + L2 * cos(angulos1.theta2));
	angulos2.theta1 = atan2(y, x) - atan2(L2 * sin(angulos2.theta2), L1 + L2 * cos(angulos2.theta2));

	// Convertir los ángulos a grados
	angulos1.theta1 *= 180.0 / M_PI;
	angulos1.theta2 *= 180.0 / M_PI;
	angulos2.theta1 *= 180.0 / M_PI;
	angulos2.theta2 *= 180.0 / M_PI;

	// Seleccionar el ángulo del hombro menor
	if (angulos1.theta1 < angulos2.theta1) {
		return angulos1;
	}
	else {
		return angulos2;
	}
}


long calcularPasosCodo(double theta2) {
	double pasosPorRev = 4096 / 360;
	long pasos = theta2 * pasosPorRev;
	return pasos;
}

int contarPasosRevolucionHombro() {
	int pasosPorRevolucion = 0;
	hombro.setMaxSpeed(500);
	//detectar estado de la leva
	bool estadoLeva = digitalRead(LEVA_HOMBRO_PIN);
	if (estadoLeva == LOW) {
		// Mover el motor hasta que se active el fin de carrera
		hombro.setSpeed(500);
		while (digitalRead(LEVA_HOMBRO_PIN) == LOW) {
			hombro.runSpeed();
		}
	}
	Serial.println("Fin de carrera detectado");
	// Guardar la posición inicial
	hombro.setCurrentPosition(0);

	// Continuar moviendo el motor para completar una revolución
	while (digitalRead(LEVA_HOMBRO_PIN) == HIGH || hombro.currentPosition() == 0) {
		hombro.setSpeed(500);
		hombro.runSpeed();
	}
	pasosPorRevolucion = hombro.currentPosition();
	// Devolver el número total de pasos
	return pasosPorRevolucion;
}

int contarPasosRevolucionCodo() {
	int pasosPorRevolucion = 0;
	codo.setMaxSpeed(500);
	//detectar estado de la leva
	bool estadoLeva = digitalRead(LEVA_CODO_PIN);
	if (estadoLeva == LOW) {
		// Mover el motor hasta que se active el fin de carrera
		codo.setSpeed(500);
		while (digitalRead(LEVA_CODO_PIN) == LOW) {
			codo.runSpeed();
		}
	}
	Serial.println("Fin de carrera detectado");
	// Guardar la posición inicial
	codo.setCurrentPosition(0);

	// Continuar moviendo el motor para completar una revolución
	while (digitalRead(LEVA_CODO_PIN) == HIGH || codo.currentPosition() == 0) {
		codo.setSpeed(500);
		codo.runSpeed();
	}
	pasosPorRevolucion = codo.currentPosition();
	// Devolver el número total de pasos
	return pasosPorRevolucion;
}



void setup() {
	Serial.begin(115200);
	SERIAL_PORT.begin(115200); // Configura la comunicación con el TMC2209

	// Configuración del driver TMC2209
	pinMode(ENABLE_PIN, OUTPUT);
	digitalWrite(ENABLE_PIN, LOW); // Habilitar el driver

	driver.begin();
	driver.toff(5);
	driver.rms_current(800); // Corriente en mA para el NEMA17
	driver.microsteps(16);   // Configurar microstepping
	driver.en_spreadCycle(false); // Deshabilitar spreadCycle, usa StealthChop
	driver.pwm_autoscale(true); // Activar auto scale PWM

	// Configuración de los pines
	pinMode(LEVA_HOMBRO_PIN, INPUT_PULLUP);
	pinMode(LEVA_CODO_PIN, INPUT_PULLUP);

	// Configuración inicial de los motores
	hombro.setMaxSpeed(1000.0);
	hombro.setAcceleration(500.0);
	codo.setMaxSpeed(1000.0);
	codo.setAcceleration(500.0);
	// Calcular los pasos por revolución para el motor del hombro
	//long pasosPorRevolucionHombro = contarPasosRevolucionHombro();
	//Serial.print("Pasos por revolucion del hombro: "); Serial.println(pasosPorRevolucionHombro);
	// calcular los pasos por revolución para el motor del codo
	//long pasosPorRevolucionCodo = contarPasosRevolucionCodo();
	//Serial.print("Pasos por revolucion del codo: "); Serial.println(pasosPorRevolucionCodo);

	// Add the motores to the MultiStepper object
	motores.addStepper(hombro); // position '0'
	motores.addStepper(codo);    // position '1'

	// Homing: Mueve los motores hasta posicionarse en el origen
	realizarHoming();

	// Espera comandos del usuario a través del puerto serie
	Serial.println("Homing completado. Listo para recibir comandos.");
}

void loop() {
	//codo.moveTo(4096);
	//codo.runToPosition();
	//delay(3000);
	//Serial.println("hombro empieza una vuelta.");
	//hombro.moveTo(-8120);
	//hombro.runToPosition();
	//Serial.println("Hombro ha dado 8100 pasos");
	//while (true)
	//{

	//}
	//hombro.moveTo(0);



	if (Serial.available() > 0) {
		String input = Serial.readStringUntil('\n');
		Serial.print("Comando recibido: "); Serial.println(input);
		moverAPosicion(input);
	}
}


