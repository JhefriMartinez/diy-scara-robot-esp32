# Manipulador Robótico SCARA de 3 GDL (RRP)

Este repositorio contiene los archivos de diseño mecánico, firmware embebido y software de control para el desarrollo de un robot manipulador tipo SCARA de 3 grados de libertad con configuración Revoluta-Revoluta-Prismática (RRP), fabricado mediante impresión 3D (PLA).

## 👥 Integrantes
* Edson Alvarez Chavez
* Jhefri Franco Martinez Cardenas
* Dave Sebastian Peñares Cuya
* Carlos Unocc Manrique

## 🛠️ Arquitectura del Sistema
* **Hardware Estructural:** Eslabones optimizados impresos en PLA (FDM).
* **Controlador Central:** Microcontrolador ESP32-DevKit V1.
* **Módulo de Potencia:** Driver PWM PCA9685 de 16 canales e interfaz I2C.
* **Actuadores:** Servomotores MG995 (alto torque) y microservo MG90S para el efector final (garra).
* **Fuente de Alimentación:** Fuente conmutada AWERS de 5VDC a 10A.

## 💻 Estructura del Repositorio
* `1_Firmware_ESP32/`: Código fuente en C++ (Arduino IDE) encargado de la recepción de tramas por UART, parsing de datos y ejecución local de la cinemática inversa embebida.
* `2_Interfaz_Python/`: Script en Python utilizando `customtkinter` para la teleoperación de consignas cartesianas y simulación predictiva 3D con `matplotlib`.
* `3_Diseño_CAD_STL/`: Modelos geométricos en formato STL listos para laminación e impresión 3D.

## 🚀 Instrucciones de Uso
1. Cargar el firmware del ESP32 ubicado en la carpeta correspondiente.
2. Energizar la etapa de potencia física con la fuente conmutada.
3. Ejecutar el script de Python `interfaz_scara.py` en la computadora maestra, seleccionar el puerto COM asignado y presionar "Conectar".
4. Introducir las coordenadas deseadas dentro del espacio de trabajo seguro (Radio: 11.75 - 16.60 cm, Z: 0.00 - 3.11 cm).
