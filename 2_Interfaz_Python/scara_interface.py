import customtkinter as ctk
import numpy as np
import serial
import serial.tools.list_ports
import threading
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D

class ScaraGUI(ctk.CTk):
    def __init__(self):
        super().__init__()
        
        # Parámetros geométricos confirmados (cm)
        self.L1 = 11.61
        self.L2 = 7.99
        self.L3 = 8.61
        self.arduino = None
        
        # Configuración de la ventana maestra
        self.title("Control e Interfaz Predictiva 3D - SCARA RRP")
        self.geometry("1100x650")
        ctk.set_appearance_mode("Dark")
        ctk.set_default_color_theme("blue")
        
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.crear_interfaz()
        
    def crear_interfaz(self):
        # Grid principal de 2 columnas
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=2)
        self.grid_rowconfigure(0, weight=1)
        
        # --------- PANEL DE CONTROL DE HARDWARE (IZQUIERDA) ---------
        self.frame_izq = ctk.CTkFrame(self, width=350, corner_radius=15)
        self.frame_izq.grid(row=0, column=0, padx=20, pady=20, sticky="nsew")
        
        self.lbl_titulo = ctk.CTkLabel(self.frame_izq, text="CONTROL DEL ROBOT", font=ctk.CTkFont(size=18, weight="bold"))
        self.lbl_titulo.pack(pady=15)
        
        # Conexión Serial
        self.frame_serial = ctk.CTkFrame(self.frame_izq)
        self.frame_serial.pack(fill="x", padx=15, pady=10)
        
        self.lbl_port = ctk.CTkLabel(self.frame_serial, text="Puerto COM:")
        self.lbl_port.grid(row=0, column=0, padx=10, pady=5)
        
        puertos = [p.device for p in serial.tools.list_ports.comports()]
        if not puertos: puertos = ["COM1", "COM2", "COM3", "COM4"]
        self.combo_puertos = ctk.CTkComboBox(self.frame_serial, values=puertos)
        self.combo_puertos.grid(row=0, column=1, padx=10, pady=5)
        
        self.btn_conectar = ctk.CTkButton(self.frame_serial, text="Conectar", command=self.toggle_conexion)
        self.btn_conectar.grid(row=1, column=0, columnspan=2, pady=10, padx=10, sticky="ew")
        
        # Entradas de Coordenadas Cartesianas (cm)
        self.frame_coord = ctk.CTkFrame(self.frame_izq)
        self.frame_coord.pack(fill="x", padx=15, pady=15)
        
        self.lbl_coord_title = ctk.CTkLabel(self.frame_coord, text="Consignas Cartesianas Target", font=ctk.CTkFont(weight="bold"))
        self.lbl_coord_title.grid(row=0, column=0, columnspan=2, pady=5)
        
        # Entrada X
        ctk.CTkLabel(self.frame_coord, text="Posición X (cm):").grid(row=1, column=0, padx=10, pady=5, sticky="w")
        self.ent_x = ctk.CTkEntry(self.frame_coord, placeholder_text="13.0")
        self.ent_x.insert(0, "13.0")
        self.ent_x.grid(row=1, column=1, padx=10, pady=5)
        
        # Entrada Y
        ctk.CTkLabel(self.frame_coord, text="Posición Y (cm):").grid(row=2, column=0, padx=10, pady=5, sticky="w")
        self.ent_y = ctk.CTkEntry(self.frame_coord, placeholder_text="5.0")
        self.ent_y.insert(0, "5.0")
        self.ent_y.grid(row=2, column=1, padx=10, pady=5)
        
        # Entrada Z
        ctk.CTkLabel(self.frame_coord, text="Posición Z (cm):").grid(row=3, column=0, padx=10, pady=5, sticky="w")
        self.ent_z = ctk.CTkEntry(self.frame_coord, placeholder_text="2.0")
        self.ent_z.insert(0, "2.0")
        self.ent_z.grid(row=3, column=1, padx=10, pady=5)
        
        # Configuración de Codo (Matriz + / -)
        ctk.CTkLabel(self.frame_coord, text="Configuración Codo:").grid(row=4, column=0, padx=10, pady=5, sticky="w")
        self.opc_codo = ctk.CTkComboBox(self.frame_coord, values=["Codo Abajo (+Th2)", "Codo Arriba (-Th2)"])
        self.opc_codo.set("Codo Abajo (+Th2)")
        self.opc_codo.grid(row=4, column=1, padx=10, pady=5)
        
        # Botón Enviar Comando
        self.btn_enviar = ctk.CTkButton(self.frame_izq, text="Validar y Enviar al Robot", fg_color="green", hover_color="darkgreen", command=self.procesar_y_enviar)
        self.btn_enviar.pack(fill="x", padx=15, pady=15)
        
        # Consola de Estado Interna
        self.txt_log = ctk.CTkTextbox(self.frame_izq, height=120)
        self.txt_log.pack(fill="x", padx=15, pady=10)
        self.log("Consola iniciada. Calibrar Home físico antes de operar.")
        
        # --------- PANEL DE VISUALIZACIÓN MATPLOTLIB (DERECHA) ---------
        self.frame_der = ctk.CTkFrame(self, corner_radius=15)
        self.frame_der.grid(row=0, column=1, padx=20, pady=20, sticky="nsew")
        
        self.fig = plt.figure(figsize=(5, 5), facecolor='#2b2b2b')
        self.ax = self.fig.add_subnet(111, projection='3d') if hasattr(self.fig, 'add_subnet') else self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor('#2b2b2b')
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.frame_der)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=10, pady=10)
        
        # Render inicial en postura de Home Seguro
        self.graficar_brazo(16.60, 0.0, 3.11, 0.0, 0.0, 0.0)

    def log(self, mensaje):
        self.txt_log.insert("end", f"[{time.strftime('%H:%M:%S')}] {mensaje}\n")
        self.txt_log.see("end")

    def toggle_conexion(self):
        if self.arduino is None:
            try:
                puerto = self.combo_puertos.get()
                self.arduino = serial.Serial(puerto, 115200, timeout=1)
                time.sleep(2)  # Reset de cortesía del ESP32
                self.log(f"Conectado con éxito a {puerto} a 115200 bps.")
                self.btn_conectar.configure(text="Desconectar", fg_color="red", hover_color="darkred")
            except Exception as e:
                self.log(f"Error de conexión: {str(e)}")
        else:
            self.arduino.close()
            self.arduino = None
            self.log("Puerto Serial cerrado limpiamente.")
            self.btn_conectar.configure(text="Conectar", fg_color=["#3a7ebf", "#1f538d"], hover_color=["#326699", "#14375e"])

    def calcular_cinematica_inversa(self, px, py, pz):
        # 1. Filtro estricto del eje vertical físico
        if pz < 0.0 or pz > 3.11:
            return False, f"Z fuera de límites estructurales (0 a 3.11 cm). Solicitado: {pz}"
            
        q3 = 3.11 - pz  # Deducción directa de la matriz
        
        # 2. Análisis del Vector de Posición Horizontal (Norma Matricial)
        radio_sq = px**2 + py**2
        radio = np.sqrt(radio_sq)
        
        # Filtro estricto de la zona de exclusión toroidal (Dona)
        if radio > 16.60 or radio < 11.75:
            return False, f"Punto fuera de zona segura. Radio: {radio:.2f} cm (Permitido: 11.75 a 16.60 cm)."
            
        # 3. Deducción de Theta 2 (Paso algebraico exacto de las láminas)
        num_th2 = radio_sq - self.L2**2 - self.L3**2
        den_th2 = 2 * self.L2 * self.L3
        cos_th2 = num_th2 / den_th2
        
        # Filtro anti NaN para evitar números complejos por truncamiento flotante
        cos_th2 = np.clip(cos_th2, -1.0, 1.0)
        
        # Selección analítica de la postura (Signo algebraico +/-)
        signo = 1.0 if "Codo Abajo" in self.opc_codo.get() else -1.0
        sin_th2 = signo * np.sqrt(1.0 - cos_th2**2)
        theta2 = np.arctan2(sin_th2, cos_th2)
        
        # 4. Deducción de Theta 1 (Desacoplamiento matricial exacto)
        num_th1 = py * (self.L2 + self.L3 * cos_th2) - px * (self.L3 * sin_th2)
        den_th1 = px * (self.L2 + self.L3 * cos_th2) + py * (self.L3 * sin_th2)
        theta1 = np.arctan2(num_th1, den_th1)
        
        # 5. Validación de los límites físicos reales por software antes de mover
        th1_deg = np.rad2deg(theta1)
        th2_deg = np.rad2deg(theta2)
        
        if th1_deg < -135.1 or th1_deg > 135.1:
            return False, f"Saturación base: Requiere {th1_deg:.1f}° (Límitado a ±135° por motor de 270°)."
        if th2_deg < -90.1 or th2_deg > 90.1:
            return False, f"Saturación codo: Requiere {th2_deg:.1f}° (Límitado a ±90° por colisión)."
            
        return True, (px, py, pz, theta1, theta2, q3)

    def procesar_y_enviar(self):
        try:
            px = float(self.ent_x.get())
            py = float(self.ent_y.get())
            pz = float(self.ent_z.get())
        except ValueError:
            self.log("Error: Formato numérico inválido en casillas.")
            return
            
        exito, resultado = self.calcular_cinematica_inversa(px, py, pz)
        
        if not exito:
            self.log(resultado)
            return
            
        px_v, py_v, pz_v, th1, th2, q3 = resultado
        
        # Actualización de la GUI 3D predictiva
        self.graficar_brazo(px_v, py_v, pz_v, th1, th2, q3)
        self.log(f"Válido -> Th1: {np.rad2deg(th1):.1f}°, Th2: {np.rad2deg(th2):.1f}°, q3: {q3:.2f} cm")
        
        # Envío asíncrono de la trama serial hacia el cálculo del firmware
        if self.arduino and self.arduino.is_open:
            trama = f"{px_v:.2f},{py_v:.2f},{pz_v:.2f}\n"
            threading.Thread(target=self.enviar_serial_async, args=(trama,)).start()
        else:
            self.log("Aviso: Coordenada simulada. Conecte el puerto serial para transmisión física.")

    def enviar_serial_async(self, trama):
        try:
            self.arduino.write(trama.encode('utf-8'))
            self.log(f"TX Serial -> {trama.strip()}")
        except Exception as e:
            self.log(f"Fallo de transmisión física: {str(e)}")

    def graficar_brazo(self, px, py, pz, th1, th2, q3):
        self.ax.clear()
        self.ax.set_facecolor('#2b2b2b')
        
        # Coordenadas cinemáticas directas de los nodos
        x0, y0, z0 = 0.0, 0.0, 0.0
        x1, y1, z1 = 0.0, 0.0, self.L1
        x2, y2, z2 = self.L2 * np.cos(th1), self.L2 * np.sin(th1), self.L1
        x3, y3, z3 = px, py, self.L1
        x4, y4, z4 = px, py, 3.11 - q3
        
        # Dibujado de los eslabones cinemáticos
        self.ax.plot([x0, x1], [y0, y1], [z0, z1], color="white", linewidth=5, label="Torre Base")
        self.ax.plot([x1, x2], [y1, y2], [z1, z2], color="#00adb5", linewidth=6, label="Eslabón 1 (L2)")
        self.ax.plot([x2, x3], [y2, y3], [z2, z3], color="#ff2e63", linewidth=4, label="Eslabón 2 (L3)")
        self.ax.plot([x3, x4], [y3, y4], [z3, z4], color="#eee", linewidth=3, label="Eje Prismático (q3)")
        
        # Marcador del Efector Final (TCP)
        self.ax.scatter([x4], [y4], [z4], color="green", s=100, zorder=10)
        
        # Formateo estético del canvas 3D
        self.ax.set_xlim([-18, 18])
        self.ax.set_ylim([-18, 18])
        self.ax.set_zlim([0, 15])
        self.ax.set_xlabel("X (cm)", color="white")
        self.ax.set_ylabel("Y (cm)", color="white")
        self.ax.set_zlabel("Z (cm)", color="white")
        self.ax.tick_params(colors="white")
        self.ax.grid(True, linestyle="--", alpha=0.3)
        self.ax.view_init(elev=25, azim=45)
        
        self.canvas.draw()

    def on_closing(self):
        if self.arduino and self.arduino.is_open:
            self.arduino.close()
        self.destroy()

if __name__ == "__main__":
    app = ScaraGUI()
    app.mainloop()