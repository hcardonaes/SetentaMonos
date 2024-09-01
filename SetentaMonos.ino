/*
 Name:		SetentaMonos.ino
 Created:	20/08/2024 16:40:34
 Author:	Ofel
*/

#include <ArticulatedTriangle2D.h>
#include <Point2D.h>
#include <Point3D.h>
#include <TrigUtils.h>
#include <TriangleSolverLib.h>
#include <TMCStepper.h>
#include <TMCStepper_UTILITY.h>
#include <ArduinoSTL.h>
#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <vector>


ArticulatedTriangle2D trig(100, 100, true);


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

double posicionActualX = 0.0;
double posicionActualY = 0.0;
double anguloActualHombro = 0.0; // Ángulo actual del hombro
double anguloActualCodo = 0.0;   // Ángulo actual del codo

struct Angles {
	double theta1;
	double theta2;
};

struct Coordenadas {
	double x;
	double y;
};

struct Punto {
	double x;
	double y;
};

bool esMovimientoValido(Coordenadas coord) {
	return coord.x >= -160 && coord.x <= 160 && coord.y >= -160 && coord.y <= 160;
}

std::vector<Coordenadas> movimientosCaballo(Coordenadas inicio) {
	std::vector<Coordenadas> movimientos;
	int dx[] = { 2, 2, -2, -2, 1, 1, -1, -1 };
	int dy[] = { 1, -1, 1, -1, 2, -2, 2, -2 };

	for (int i = 0; i < 8; ++i) {
		Coordenadas nuevoMovimiento = { inicio.x + dx[i] * 40, inicio.y + dy[i] * 40 };
		if (esMovimientoValido(nuevoMovimiento)) {
			movimientos.push_back(nuevoMovimiento);
		}
	}
	return movimientos;
}

void moverCaballo(String comandoInicio, String comandoFin) {
	Coordenadas inicio = calcularCoordenadasDesdeCentro(comandoInicio);
	Coordenadas fin = calcularCoordenadasDesdeCentro(comandoFin);

	std::vector<Coordenadas> movimientos = movimientosCaballo(inicio);

	bool movimientoValido = false;
	for (const auto& movimiento : movimientos) {
		if (abs(movimiento.x - fin.x) < 1e-6 && abs(movimiento.y - fin.y) < 1e-6) {
			movimientoValido = true;
			break;
		}
	}

	if (movimientoValido) {
		Serial.print("Moviendo el caballo de "); Serial.print(comandoInicio); Serial.print(" a "); Serial.println(comandoFin);
		moverAPosicion(comandoFin);
	}
	else {
		Serial.println("Movimiento inválido para el caballo.");
	}
}

long pasosMotores[2]; // Array para almacenar las pasosMotores objetivo

//Coordenadas calcularCoordenadasDesdeCentro(String notacion) {
//	// Dimensiones del tablero
//	//constexpr double ladoTablero = 323.25;
//	constexpr double ladoTablero = 320;
//	constexpr double tamanoCasilla = ladoTablero / 8.0;
//
//	// Coordenadas del centro del tablero
//	constexpr double centroTablero = ladoTablero / 2.0;
//
//	// Convertir notación de ajedrez a índices de fila y columna
//	int columna = notacion[0] - 'a'; // Columna [a-h] -> [0-7]
//	int fila = notacion[1] - '1';    // Fila [1-8] -> [0-7]
//
//	// Calcular coordenadas desde el centro del tablero
//	double x = (columna + 0.5) * tamanoCasilla - centroTablero;
//	double y = (fila + 0.5) * tamanoCasilla - centroTablero;
//
//	Coordenadas coord = { x, y };
//	return coord;
//}

Coordenadas calcularCoordenadasDesdeCentro(String comando) {
	int columna = comando[0] - 'a'; // Columna [a-h]
	int fila = comando[1] - '1';    // Fila [1-8]
	double x = (columna - 3.5) * 40; // Ajuste para centrar en el tablero
	double y = (fila - 3.5) * 40;    // Ajuste para centrar en el tablero
	return { x, y };
}


std::vector<Punto> bresenham(double x0, double y0, double x1, double y1) {
	std::vector<Punto> puntos;
	int dx = abs(x1 - x0);
	int dy = abs(y1 - y0);
	int sx = (x0 < x1) ? 1 : -1;
	int sy = (y0 < y1) ? 1 : -1;
	int err = dx - dy;

	while (true) {
		puntos.push_back({ x0, y0 });
		if (x0 == x1 && y0 == y1) break;
		int e2 = 2 * err;
		if (e2 > -dy) {
			err -= dy;
			x0 += sx;
		}
		if (e2 < dx) {
			err += dx;
			y0 += sy;
		}
	}
	return puntos;
}

double suavizarAngulo(double nuevoAngulo, double anguloActual) {
	double diferencia = nuevoAngulo - anguloActual;
	if (diferencia > 180) {
		nuevoAngulo -= 360;
	}
	else if (diferencia < -180) {
		nuevoAngulo += 360;
	}
	return nuevoAngulo;
}

void realizarHoming() {

	// Homing para el codo
	bool estadoLevaCodo = digitalRead(LEVA_CODO_PIN);
	if (estadoLevaCodo == LOW) {
		// Mover el motor hasta que se active el fin de carrera
		codo.setSpeed(800);
		while (digitalRead(LEVA_CODO_PIN) == LOW) {
			codo.runSpeed();
		}
	}
	else {
		codo.setSpeed(-800);
		while (digitalRead(LEVA_CODO_PIN) == HIGH) {
			codo.runSpeed();
		}
	}
	codo.setCurrentPosition(0); // Establece la posición actual transitoria como cero

	// Homing para el hombro
	bool estadoLevaHombro = digitalRead(LEVA_HOMBRO_PIN);
	bool levaInicial = estadoLevaHombro;
	if (estadoLevaHombro == LOW) {
		// Mover el motor hasta que se active el fin de carrera
		hombro.setSpeed(-800);
		while (digitalRead(LEVA_HOMBRO_PIN) == LOW) {
			hombro.runSpeed();
		}
	}
	else {
		hombro.setSpeed(800);
		while (digitalRead(LEVA_HOMBRO_PIN) == HIGH) {
			hombro.runSpeed();
		}
	}
	hombro.setCurrentPosition(0); // Establece la posición actual como cero
	////if (levaInicial==0)	{pasosMotores[0] = 0;}
	////else{pasosMotores[0] = 0;}
	pasosMotores[0] = 3940;
	pasosMotores[1] = 1575;
	//// Mover los motores a la posición deseada
	motores.moveTo(pasosMotores);
	motores.runSpeedToPosition();
	//// Establece la posición actual como cero
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

	// Generar puntos intermedios usando el algoritmo de Bresenham
	std::vector<Punto> puntos = bresenham(posicionActualX, posicionActualY, x, y);

	// Mover el efector a través de los puntos generados
	for (const auto& punto : puntos) {
		trig.Target.X = punto.x;
		trig.Target.Y = punto.y;
		trig.SolveReverse();
		moverMotores();
	}

	// Actualizar la posición actual del efector
	posicionActualX = x;
	posicionActualY = y;
}

long calcularPasosHombro(double theta1) {
	double pasosPorGradoHombro = 8120 / 360;
	long pasos = theta1 * pasosPorGradoHombro;
	return pasos;
}

long calcularPasosCodo(double theta2) {
	double pasosPorGradoCodo = 4096 / 360;
	long pasos = -theta2 * pasosPorGradoCodo;
	return pasos;
}

void moverMotores() {
	// Verificar si la solución es válida
	if (isnan(trig.AbsoluteAngle1) || isnan(trig.RelativeAngle12)) {
		Serial.println("Error: No se pudo calcular los ángulos para la posición objetivo.");
		return;
	}

	Angles angulos;
	angulos.theta1 = degrees(trig.AbsoluteAngle1);
	angulos.theta2 = degrees(trig.RelativeAngle12);

	// Suavizar cambios bruscos en los ángulos
	//angulos.theta1 = suavizarAngulo(angulos.theta1, anguloActualHombro);
	//angulos.theta2 = suavizarAngulo(angulos.theta2, anguloActualCodo);

	Serial.print("Angulos: ("); Serial.print(angulos.theta1); Serial.print(", "); Serial.print(angulos.theta2); Serial.println(")");

	// Convertir ángulos a pasos de motor (en pasos)
	pasosMotores[0] = calcularPasosHombro(angulos.theta1);
	pasosMotores[1] = calcularPasosCodo(angulos.theta2);
	Serial.print("Pasos: ("); Serial.print(pasosMotores[0]); Serial.print(", "); Serial.print(pasosMotores[1]); Serial.println(")");

	// Mover los motores a las pasosMotores calculadas
	motores.moveTo(pasosMotores);
	motores.runSpeedToPosition();

	// Actualizar los ángulos actuales
	anguloActualHombro = angulos.theta1;
	anguloActualCodo = angulos.theta2;
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
	hombro.setMaxSpeed(800);
	hombro.setAcceleration(800);
	codo.setMaxSpeed(800);
	codo.setAcceleration(800);

	// Add the motores to the MultiStepper object
	motores.addStepper(hombro); // position '0'
	motores.addStepper(codo);    // position '1'

	// Homing: Mueve los motores hasta posicionarse en el origen
	realizarHoming();
	delay(5000);
	// Ajustar la posición inicial del efector al centro de la casilla d4
	Coordenadas coordInicial = calcularCoordenadasDesdeCentro("d4");
	posicionActualX = coordInicial.x;
	posicionActualY = coordInicial.y;
	Serial.print("Posicion inicial: ("); Serial.print(posicionActualX); Serial.print(", "); Serial.print(posicionActualY); Serial.println(")");

	// Mover el efector al centro de la casilla d4
	trig.Target.X = posicionActualX;
	trig.Target.Y = posicionActualY;
	trig.SolveReverse();
	moverMotores();

	// Espera comandos del usuario a través del puerto serie
	Serial.println("Homing completado. Listo para recibir comandos.");
}


void loop() {
	if (Serial.available() > 0) {
		String input = Serial.readStringUntil('\n');
		Serial.print("Comando recibido: "); Serial.println(input);

		if (input.startsWith("caballo")) {
			String comandoInicio = input.substring(8, 10);
			String comandoFin = input.substring(11, 13);
			moverCaballo(comandoInicio, comandoFin);
		}
		else {
			moverAPosicion(input);
		}
	}
}



