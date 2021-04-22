# PracticoMicroprocesadores
La idea es utilizar  dos sensores, LM35 y PT1000 leerlos en el ADC y enviarlos por la UART con una MEF que controle todo el proceso.
El usuario pulsa el botón y el equipo comienza la adquisición y envío datos por la UART hasta que se pulse nuevamente el botón.
Los datos se van a recepcionar en la PC, en una interfaz hecha en QT(C++). 
La máquina de estados va a controlar todo el proceso (Adquisición, conversión a valores de ingenieria, envío de datos, ACK de la PC).

Periféricos: 
UART, ADC, GPIO

Descripción de estados MEF:

STANDBY: Estado inicial de la MEF, se inicializa la máquina en ese estado, el sistema lee las entradas digitales las procesa en un DEBOUNCE,(otra MEF) y así hasta que se detecte un botón pulsado. 
Si se pulsó el botón, el flag STARTDATA pasa a TRUE y la máquina cambia de estado.
LEERSENSORES: Se inicia la conversión del ADC hasta que se dispara la interrupción “HAL_ADC_ConvCpltCallback” dentro se leen los datos adquiridos y se almacenan en un buffer. Una vez terminado, se levanta el flag RawDataReady.

CONVERTIRDATA: Toma los valores crudos del buffer y los convierte en valores de ingeniería, revisa si los datos son consistentes con los rangos de medición de los sensores, si es así levanta el flag dataReady, sino levanta el flag badValue.

ENVIARDATA: Envía los datos por la UART, si el tiempo de timeOUT se cumple intenta reenviar los datos nuevamente. Si los datos son enviados con éxito, levanta el flag SendDAtoOK.

LEERDATA: Lee la UART esperando el ACK desde la PC, si recibe el ACK, levanta el flag ACKPC, en caso contrario, el falg NOACKPC es levantado.

En cualquiera de los estados si se presiona el botón, el flag STARTDATA pasa a FALSE y la máquina retorna a STANDBY.

Módulos de software.

protocolo.h  - protocolo.c leer y escribir en la UART mediante un protocolo definido.
myDelay.h  - myDelay.c  Rutinas de manejo de tiempos, delay Non-Blocking
debouncebuttonuser.h - debouncebuttonuser.c  MEF para el manejo del botón.
main.h  - main.c  Rutina principal, MEF principal.
