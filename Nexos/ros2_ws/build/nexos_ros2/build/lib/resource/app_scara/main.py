import tkinter as tk
import ttkbootstrap as ttk
from ttkbootstrap.scrolled import ScrolledFrame
from resource.app_scara.src.cinematica_directa import CinematicaDirecta
from resource.app_scara.src.cinematica_inversa import CinematicaInversa
import os
from tkinter import PhotoImage

class MainApp():
    def __init__(self, req_motor_ang1_publisher, req_motor_ang2_publisher, efector_final, tipo_pose, tipo_vector, nodo) -> None:
        self.req_motor_ang1_publisher = req_motor_ang1_publisher
        self.req_motor_ang2_publisher = req_motor_ang2_publisher
        self.efector_final = efector_final
        self.tipo_pose = tipo_pose
        self.tipo_vector = tipo_vector
        self.ventana = None
        self.nodo = nodo
        #self.iniciar_app()

    def iniciar_app(self):
        try:
            ## Configuracion de la ventana
            self.ventana = ttk.Window(themename= "darkly")
            self.ventana.title("NexOS")                       # Segun las guidelines de GNOME
            home_dir = os.path.expanduser('~')
            directorio_actual = home_dir+"/Nexos/ros2_ws/src/nexos_ros2/resource/app_scara/images"
            ruta_nexos_icono = os.path.join(directorio_actual, "nexos_icono.png")
            icono = PhotoImage(file=ruta_nexos_icono, master= self.ventana)
            self.ventana.iconphoto(False, icono)
            ancho_pantalla = self.ventana.winfo_screenwidth()
            alto_pantalla = self.ventana.winfo_screenheight()
            self.ventana.geometry(f"{ancho_pantalla//2-100}x{alto_pantalla}+0+0") # +0+0 es +x+y pone la ventana en la esquina superior izquierda

            ## Crear estilo personalizado para el botón con texto en negrita
            estilo = ttk.Style()
            estilo.configure("TButton",
                            font=("TimesNewRoman", 12, "bold"))  # Mantener otras configuraciones del estilo "info"
            
            ## Crear frame principal
            frame_scroll = ScrolledFrame(master= self.ventana, autohide= True)
            frame_scroll.pack(fill= ttk.BOTH, expand= True, padx= 10, pady= 10)
            frame_contenedor = tk.Frame(master= frame_scroll)
            frame_contenedor.pack(expand=True)

            ## Crear objeto CinematicaInversa
            cinematica_inversa = CinematicaInversa(tk= tk, ttk= ttk, frame_contenedor= frame_contenedor, ventana= self.ventana, efector_final= self.efector_final, tipo_pose= self.tipo_pose, nodo= self.nodo)

            ## Crear objeto CinematicaDirecta
            cinematica_directa = CinematicaDirecta(tk= tk, ttk= ttk, frame_contenedor= frame_contenedor, ancho_pantalla= ancho_pantalla, req_motor_ang1_publisher= self.req_motor_ang1_publisher, req_motor_ang2_publisher= self.req_motor_ang2_publisher, tipo_vector= self.tipo_vector, nodo= self.nodo)

            self.ventana.mainloop()

        except KeyboardInterrupt:
            print("Se ha interrumpido desde la consola")

        except Exception as e:
            print(f"No se pudo ejecutar el codigo, se produjo la siguiente excepcion:\nTipo: {type(e).__name__}\nLinea_de_codigo: {e.__traceback__.tb_lineno}\nExcepcion: {e}")
