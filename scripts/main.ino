#define STEP 4
#define DIR 5

const float resolution = 1.8;

void setup()
{
    Serial.begin(9600); // Inicializa la comunicaci´on serie a 9600 bps
    pinMode(STEP, OUTPUT);
    pinMode(DIR, OUTPUT);
}

void loop()
{
    // Si hay alg´un mensaje disponible por el BUS de serie, se ejecuta
    if (Serial.available() > 0)
    {
        // Lee el mensaje enviado desde Python
        String mensaje = Serial.readStringUntil('\n');

        Serial.print("Mensaje recibido desde Python: ");
        Serial.println(mensaje);

        // Inicializa las variables para almacenar los grados y tiempo de giro
        int angle = 0;
        int turning_time = 0;

        // Mapea el mensaje para extraer los grados y el tiempo de giro
        int indexG = mensaje.indexOf('G');

        if (indexG != -1)
        {
            angle = mensaje.substring(indexG + 1, mensaje.indexOf('T')).toInt();
        }

        int indexT = mensaje.indexOf('T');

        if (indexT != -1)
        {
            turning_time = mensaje.substring(indexT + 1).toInt();
        }

        // Se envía un resumen de los datos recibidos
        Serial.print("Grados recibidos: ");
        Serial.println(angle);
        Serial.print("Tiempo de giro recibido: ");
        Serial.print(turning_time);
        Serial.println(" ms");

        // Con los datos recibidos se procede a realizar el giro de motor
        move(angle, turning_time);
    }
}

void move(int angle, int turning_time)
{
    digitalWrite(DIR, HIGH); // Activamos un sentido de giro

    int steps = angle / resolution;

    Serial.print("Se ejecutarán ");
    Serial.print(steps);
    Serial.println(" pasos de motor");

    int ms = (turning_time * resolution) / (2 * angle);

    for (int i = 0; i < steps; i++)
    {
        digitalWrite(STEP, HIGH); // Nivel alto
        delay(ms);                // Durante un tiempo "ms" en milisegundos
        digitalWrite(STEP, LOW);  // Nivel bajo
        delay(ms);                // Durante un tiempo "ms" en milisegundos
    }

    delay(2000); // Demora de 2 segundos
}
