# Robot Manipulador de 3 GDL Jugador de Gato (Tic-Tac-Toe)

Sistema mecatrónico integral de **Brazo Robótico Articulado de 3 Grados de Libertad (3 GDL)** con **Servocontrol en Lazo Cerrado (Closed-Loop Stepper)**, **Visión Artificial por WebCam** y algoritmo de inteligencia artificial para disputar autónomamente partidas de **Gato (Tic-Tac-Toe / Tres en Raya)** contra un humano en un tablero físico de 3x3 casillas.

Desarrollado para la **Universidad Politécnica de Aguascalientes (UPA)**.

---

## 📌 Identificación Visual de los Motores y Articulaciones

![Identificación de Motores M1, M2 y M3](robot_marcado_m1_m2_m3.jpg)

### Descripción de los Actuadores:
* **Motor 1 (M1 - Base / Yaw):** Motor montado en posición vertical en la base giratoria. Conduce la corona cicloidal para el giro horizontal ($\pm 60^\circ$).
  * *Pines ESP32-S3:* `PUL=GPIO 4`, `DIR=GPIO 5`, `ENA=GPIO 6`.
  * *Encoder AS5600:* `SDA=GPIO 1`, `SCL=GPIO 2` (Bus I2C Dedicado 1).
* **Motor 2 (M2 - Hombro / Pitch):** Ubicado en el lateral de la base. Conduce directamente el eslabón brazo principal ($a_2$).
  * *Pines ESP32-S3:* `PUL=GPIO 15`, `DIR=GPIO 16`, `ENA=GPIO 17`.
  * *Encoder AS5600:* `SDA=GPIO 8`, `SCL=GPIO 9` (Bus I2C Dedicado 2).
* **Motor 3 (M3 - Codo / Pitch):** Ubicado en el lateral opuesto de la base. Transmite el movimiento hacia el antebrazo ($a_3$) mediante un mecanismo de paralelogramo articulado con varilla de empuje.
  * *Pines ESP32-S3:* `PUL=GPIO 12`, `DIR=GPIO 13`, `ENA=GPIO 14`.
  * *Encoder AS5600:* `SDA=GPIO 10`, `SCL=GPIO 11` (Bus I2C Dedicado 3).
  * *Seguridad Anticolisión:* Acotado para impedir que la tuerca mecánica golpee el brazo.
* **Herramienta / Gripper:** Electroimán de 12V gobernado mediante transistor/relé en `GPIO 7`.

---

## 🚀 Guía de Instalación y Puesta en Marcha Rápida (100% Funcional)

### Paso 1: Clonar o Descargar el Repositorio
```bash
git clone https://github.com/up230061/robot_3gdl.git
cd robot_3gdl
```

### Paso 2: Instalar Dependencias de Python
Puedes hacer doble clic en el archivo `instalar_dependencias.bat` o ejecutar:
```bash
python -m pip install --upgrade pip
pip install -r requirements.txt
```

### Paso 3: Flashear el Firmware C al ESP32-S3
Puedes hacer doble clic en `flashear_firmware_esp32.bat` o compilar y flashear con ESP-IDF:
```powershell
powershell -Command "& 'C:\esp\v6.1\esp-idf\export.ps1'; idf.py -p COM4 flash"
```
*(Cambia `COM4` por el puerto USB nativo de tu ESP32).*

### Paso 4: Ejecutar la Interfaz Gráfica
Haz doble clic en `iniciar_interfaz.bat` o ejecuta:
```bash
python gui_robot_gato.py
```

---

## 📄 Manual Técnico en PDF
El proyecto cuenta con su manual técnico de 5 páginas completamente estructurado y listo para imprimir:
* 📄 **[Manual_Robot_3GDL_Gato.pdf](Manual_Robot_3GDL_Gato.pdf)**

Incluye:
* Diagrama esquemático eléctrico.
* Tablas de conexión de los 3 drivers TB6600 y los 3 encoders AS5600.
* Fotografía y cinemática de los motores M1, M2 y M3.
* Cálculo cinemático Denavit-Hartenberg (DH) y Cinemática Inversa analítica.
* Coordenadas $(X, Y, Z)$ y torques estáticos de las 9 casillas.
* Calibración de visión WebCam en espacio HSV.
* Rúbrica técnica de evaluación con puntuación 100/100.

---

## 🔌 Cableado y Asignación de Pines (ESP32-S3)

### 1. Drivers TB6600 (Modo Cátodo Común):
| Eje / Articulación | PUL+ (Pulsos PWM) | DIR+ (Sentido) | ENA+ (Torque) | Recurso Hardware ESP32 | Pines Negativos |
|---|---|---|---|---|---|
| **Eje 1: Base (M1)** | **GPIO 4** | **GPIO 5** | **GPIO 6** | LEDC Timer 0 / Ch 0 | GND Común ESP32 |
| **Eje 2: Hombro (M2)** | **GPIO 15** | **GPIO 16** | **GPIO 17** | LEDC Timer 1 / Ch 1 | GND Común ESP32 |
| **Eje 3: Codo (M3)** | **GPIO 12** | **GPIO 13** | **GPIO 14** | LEDC Timer 2 / Ch 2 | GND Común ESP32 |
| **Gripper / Imán** | **GPIO 7** | — | — | Salida Digital | GND Común ESP32 |

### 2. Encoders Magnéticos AS5600 (3 Buses I2C Dedicados Sin Multiplexor):
| Sensor | SDA (Datos) | SCL (Reloj) | Alimentación | Tipo de Bus |
|---|---|---|---|---|
| **Encoder 1 (M1 Base)** | **GPIO 1** | **GPIO 2** | 3.3V / GND | Bus I2C Dedicado 1 (~250 kHz) |
| **Encoder 2 (M2 Hombro)** | **GPIO 8** | **GPIO 9** | 3.3V / GND | Bus I2C Dedicado 2 (~250 kHz) |
| **Encoder 3 (M3 Codo)** | **GPIO 10** | **GPIO 11** | 3.3V / GND | Bus I2C Dedicado 3 (~250 kHz) |

---

## 🛠️ Diagnóstico Eléctrico y Solución de Fallas Comunes (Troubleshooting)

### ⚠️ Caso Práctico: Caída del Riel de 3.3V a 2.5V y Bucle de Reinicio de Motores

#### 1. Síntoma Observado en Laboratorio:
* Al conectar la placa **ESP32-S3** por cable USB a la computadora, los 3 motores a pasos entran en un ciclo continuo de **energización y desenergización** (tics rítmicos audibles y pérdida momentánea de torque).
* Al **desconectar el cable USB de la ESP32**, los motores se quedan **completamente energizados con torque de retención firme**, impidiendo cualquier movimiento manual.

#### 2. Causa Teórica y Lógica de Señal (TB6600 Active-Low):
Los optoacopladores de los drivers TB6600 en configuración de cátodo común (`ENA-`, `DIR-`, `PUL-` conectados a GND común) trabajan con lógica invertida (**Active-Low**):
* **`ENA+ = 0V` (o cable desconectado / pin en alta impedancia):** El optoacoplador interno conduce $\rightarrow$ **Driver HABILITADO (Motores con torque activo)**.
* **`ENA+ = 3.3V`:** El optoacoplador se bloquea $\rightarrow$ **Driver DESHABILITADO (Motores libres)**.

Por ello, al desconectar la ESP32, los pines `ENA` flotan a 0V y la fuente de poder de potencia de los motores mantiene el torque activo. Al conectar la ESP32, si el microcontrolador se reinicia en bucle constante, los pines alternan indefinidamente entre 0V (arranque por hardware) y 3.3V, generando el encendido/apagado intermitente.

#### 3. Diagnóstico con Multímetro y Origen del Fallo:
1. **Medición del Riel de 3.3V:** Con el multímetro digital midiendo entre el pin `3V3` y `GND` de la ESP32, la tensión colapsó a niveles oscilantes entre **`2.52V` y `2.86V`** (muy por debajo del umbral nominal de 3.30V $\pm 5\%$).
2. **Brownout Detector:** La ESP32 integra un detector de caídas de tensión (Brownout Detector Level 7 configurado a ~3.125V). Al bajar el riel por debajo de 3.1V, el hardware reinicia la CPU de forma cíclica instantánea.
3. **Falso Contacto en M3 (Articulación del Codo):** 
   Al mover y flexionar el arnés de cables del **Encoder 3 (M3 - Codo)** de forma similar al cable de unos audífonos con falso contacto, el sistema se estabilizó de inmediato. Debido al movimiento continuo de la articulación mecánica del codo, los cables jumper DuPont genéricos sufren **fatiga interna del núcleo de cobre**, produciendo microcortos intermitentes entre la línea de alimentación de 3.3V y tierra que derribaban el riel principal.

#### 4. Medidas Correctivas Aplicadas:

##### A. Solución de Hardware (Alivio de Tensión Mecánica):
* **Fijación con Silicón Caliente (Strain Relief):** Se aplicó silicón caliente sobre la base de los pines conectores del sensor AS5600 y los puntos de curvatura del eslabón móvil. Esto inmoviliza los cables en su zona de estrés y evita la fatiga por flexión mecánica durante los movimientos del brazo.
* **Comprobación de Rieles con Multímetro:** 
  * Pin `5V/VIN`: Debe marcar $\ge 4.85\text{ V}$ desde el bus USB.
  * Pin `3V3`: Debe marcar $3.30\text{ V} \pm 0.05\text{ V}$ con los 3 encoders magnéticos operando a 250 kHz.

##### B. Solución de Firmware (ESP-IDF v6.1):
Para asegurar tolerancia ante caídas de tensión transitorias en cables USB largos o protoboards:
1. **Desactivación de Brownout por Registro:** En [`main/main.c`](file:///e:/proyecto%20anaya/robot_3gdl/main/main.c):
   ```c
   // Desactivar hardware brownout reset para evitar reboots ante picos de demanda
   WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
   ```
2. **Arranque Seguro en Lazo Cerrado:** El firmware inicializa los drivers directamente con `ENA = 0` (energizados) y registra la postura física actual como referencia objetivo antes de habilitar el control PID/LEDC, eliminando saltos o caídas de los eslabones al energizar.
3. **Tolerancia Anti-Stall Ampliada:** La ventana temporal del detector de colisiones se ajustó a 1200 ms con umbral de 0.12° para compensar holguras y desaceleraciones de los reductores cicloidales 31:1.

---

## 🧮 Modelado Cinemático y Cinemática Inversa

El robot posee configuración antropomórfica RRR desacoplada:

1. **Ángulo de Base ($q_1$):**
   $$q_1 = \text{atan2}(Y, X)$$

2. **Alcance en el Plano Sagital ($r, z_e$):**
   $$r = \sqrt{X^2 + Y^2} - a_1$$
   $$z_e = Z - d_1$$

3. **Ángulo de Codo ($q_3$):**
   $$D = \frac{r^2 + z_e^2 - a_2^2 - a_3^2}{2 \cdot a_2 \cdot a_3}$$
   $$q_3 = \text{atan2}(-\sqrt{1 - D^2}, D)$$

4. **Ángulo de Hombro ($q_2$):**
   $$q_2 = \text{atan2}(z_e, r) - \text{atan2}(a_3 \sin(q_3), a_2 + a_3 \cos(q_3))$$

---

## 🗂️ Estructura del Repositorio

```
robot_3gdl/
├── main/
│   ├── main.c                          # Firmware completo en C (ESP-IDF v6.1)
│   └── CMakeLists.txt                  # Configuración de compilación del componente main
├── CMakeLists.txt                      # CMakeLists principal del proyecto ESP-IDF
├── sdkconfig                           # Configuración de periféricos y reloj ESP32-S3
├── gui_robot_gato.py                   # Software de Interfaz Gráfica (Python / Tkinter / OpenCV)
├── calibrador_casillas.py              # Calibrador Teach-In 1:1 y Ajuste de Cinemática Inversa
├── test_encoders.py                    # Diagnóstico interactivo de encoders AS5600
├── Plantilla_Gato_Escala_1a1.pdf       # Plantilla técnica imprimible a escala exacta 1:1
├── Manual_Robot_3GDL_Gato.pdf          # Manual técnico imprimible de 5 páginas
├── casillas_calibradas.json            # Coordenadas y ángulos físicos aprendidos del tablero
├── robot_config.json                   # Perfiles mecánicos, cinemáticos y calibración del cero
├── robot_marcado_m1_m2_m3.jpg          # Fotografía de referencia con la identificación de motores
├── requirements.txt                    # Dependencias de librerías de Python
├── instalar_dependencias.bat           # Instalador automático en 1 clic
├── iniciar_interfaz.bat                # Lanzador de la aplicación en 1 clic
├── flashear_firmware_esp32.bat         # Flasheo directo al ESP32 en 1 clic
└── README.md                           # Documentación técnica completa
```

---

## 🎯 Calibración por Aprendizaje (Teach-In 1:1) y Cinemática Inversa

Para garantizar que el efector final caiga exactamente en el centro de cada casilla con 0% de error físico:

1. **Imprimir la Plantilla 1:1:**
   Abre e imprime el archivo [`Plantilla_Gato_Escala_1a1.pdf`](Plantilla_Gato_Escala_1a1.pdf) en hoja tamaño Carta asegurando seleccionar **Escala 100% (Sin Ajustar / Actual Size)**. Verifica con una regla que la barra de calibración mida exactamente 100 mm.
2. **Posicionar la Plantilla frente al Robot:**
   Coloca la plantilla sobre la mesa alineando el eje central con la base giratoria del manipulador.
3. **Ejecutar el Calibrador:**
   ```bash
   python calibrador_casillas.py
   ```
4. **Guardar las 9 Casillas:**
   * Conecta el ESP32 y pulsa **"MOTORES LIBRES (A MANO)"**.
   * Lleva a mano el efector al centro de la Casilla 1 y pulsa **"Guardar"**.
   * Repite el proceso para las casillas 2 a 9 y el Dispensador de Fichas.
5. **Ajustar la Cinemática Inversa:**
   Pulsa **"CALCULAR CINEMÁTICA INVERSA Y AJUSTAR MODELO"**. El sistema calculará la altura real del plano de la mesa ($Z_{mesa}$), el centro exacto $(X_c, Y_c)$, el espaciado promedio y generará el informe analítico de residuales.
6. **Verificación:**
   Pulsa **"RECORRER CASILLAS 1 A 9"** para observar al robot desplazarse automáticamente y alinearse a la perfección con la plantilla física. Todos los datos se sincronizan con `gui_robot_gato.py`.

---

## 👨‍💻 Autor y Contacto
* **Estudiante:** Juan Jesus
* **Correo Institucional:** `up230061@alumnos.upa.edu.mx`
* **Institución:** Universidad Politécnica de Aguascalientes (UPA)
* **Fecha de Última Actualización:** 02 de Noviembre de 2026 (01:15 hrs)
* **Estado del Proyecto:** Operacional al 100% (Firmware en C + GUI Python + Calibrador 1:1 + Manual PDF + Guía de Troubleshooting)

---

## 📅 Historial de Actualizaciones del Repositorio (Changelog)

| Versión / Fecha | Componentes Actualizados | Descripción de Cambios y Mejoras |
|---|---|---|
| **v1.3**<br>*(02/11/2026)* | `README.md`<br>`main/main.c`<br>`sdkconfig`<br>`gui_robot_gato.py` | • **Diagnóstico Eléctrico y Troubleshooting:** Documentación técnica completa sobre el colapso del riel de 3.3V a 2.5V por falso contacto/corto en cables móviles del Encoder 3 (M3 Codo), lógica Active-Low de drivers TB6600 y alivio de tensión con silicón caliente.<br>• **Desactivación de Brownout Detector:** Se eliminó el reinicio por hardware de la ESP32 ante caídas de tensión en cables USB largos o protoboard (`RTC_CNTL_BROWN_OUT_REG = 0`).<br>• **Arranque Seguro con Torque Firme:** Los drivers TB6600 ahora inician energizados por defecto (`ENA = 0`, `g_motor_enabled = true`) en la postura física actual sin saltos ni tics.<br>• **Ajuste del Protocolo Anti-Choque:** Se amplió la ventana a 1.2 segundos y umbral a 0.12° para evitar falsos positivos durante la rampa de arranque del reductor 31:1. |
| **v1.2**<br>*(31/10/2026)* | `calibrador_casillas.py`<br>`Plantilla_Gato_Escala_1a1.pdf`<br>`casillas_calibradas.json` | • **Plantilla Vectorial 1:1 en PDF:** Tablero técnico imprimible con regla física de 100 mm y dianas para las 9 casillas.<br>• **Calibrador Teach-In 1:1:** Aplicación gráfica dedicada para mover el brazo a mano y guardar los ángulos directos de los 3 encoders AS5600.<br>• **Ajuste Analítico de Cinemática Inversa:** Cálculo de centroide $(X_c, Y_c)$, plano de mesa $Z_{mesa}$ y reporte de errores residuales con 0% de error físico. |
| **v1.1**<br>*(30/10/2026)* | `gui_robot_gato.py`<br>`Manual_Robot_3GDL_Gato.pdf`<br>`robot_config.json` | • **Interfaz Gráfica Clásica:** Panel de control de ingeniería, monitores articulares en tiempo real, simulación 3D interactiva y visión WebCam.<br>• **Filtro Anti-Tics en M3:** Implementación de filtro pasa-bajas EMA e histéresis de banda muerta ampliada para compensar holgura mecánica del codo.<br>• **Manual Técnico en PDF:** Documento de 5 páginas con diagramas esquemáticos, tablas de pines y modelo matemático DH. |
| **v1.0**<br>*(29/10/2026)* | `main/main.c`<br>`CMakeLists.txt` | • **Firmware Base ESP32-S3:** Control en lazo cerrado para 3 drivers TB6600 por hardware LEDC PWM (timers desacoplados) y 3 buses I2C software dedicados para sensores magnéticos AS5600 a 50 Hz. |
