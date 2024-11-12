from tkinter import filedialog
import tkinter.scrolledtext as scrolledtext
import multiprocessing as mp
import os

class CinematicaInversa():
    '''
    Estructura de la aplicacion que sera usada para la cinematica inversa.

    Contiene las siguientes funciones:
    * cinematica_inversa: Se encarga de publicar en el topic efector_final, las posiciones x, y, z y la orientacion z que el usuario pase por parametros. Parametros:
        1. pos_x: Posicion x del efector final.
        2. pos_y: Posicion y del efector final.
        3. pos_z: Posicion z del efector final.
        4. or_z: Orientacion z del efector final.
    * ejecutar_script: Se encarga de ejectuar lo que el usuario tenga escrito dentro del recuadro para escribir codigo, solo se ejecutar cuando se le da al boton de ejecutar codigo. No tiene parametros.
    * abrir_archivo_python: Funcion para leer el codigo de un archivo python y escribirlo en el recuadro para la ejecucion. No tiene parametros.
    * guardar_archivo_python: Funcion para guardar el codigo que se tenga escrito en el recuadro de ejecucion. No tiene parametros.
    * crear_menu_archivos: Funcion que crea el menu de archivos para guardar y abrir archivos python. No tiene parametros.
    * crear_cinematica_inversa: Funcion para crear toda la parte grafica y funcional de la cinematica inversa. Parametros:
        1. frame_contenedor: Frame dentro del que esta toda la parte grafica de la aplicacion.
    '''
    def __init__(self, tk, ttk, frame_contenedor, ventana, efector_final, tipo_pose, nodo) -> None:
        self.script_codigo = None
        self.tk = tk
        self.ttk = ttk
        self.ventana = ventana
        self.efector_final = efector_final
        self.tipo_pose = tipo_pose
        self.process = None
        self.nodo = nodo
        self.mensaje_esp32 = None
        self.urosok2_count = 0
        self.urosok2_ant = 0
        self.crear_menu_archivos()
        self.crear_cinematica_inversa(frame_contenedor)

    def cinematica_inversa(self, pos_x= 0.0, pos_y= 0.0, pos_z= 25.4, or_z= 0.0):
        try:
            msg = self.tipo_pose()
            msg.position.x = pos_x
            msg.position.y = pos_y
            msg.position.z = pos_z
            msg.orientation.z = or_z
            self.efector_final.publish(msg)
            self.nodo.get_logger().info("Cinematica inversa enviada")
        except Exception as e:
            self.nodo.get_logger().error(f"No se pudo ejecutar la cinematica_inversa: {e}")

    def ejecutar_script(self):
        try:
            if self.process is None: 
                self.process = mp.Process(target= exec(self.script_codigo.get("1.0", self.tk.END))) # La especificación "1.0" indica que comienza desde la primera línea y el primer carácter, y tk.END indica que va hasta el final del contenido
                self.process.start()
                self.process.join()
            elif not self.process.is_alive():
                self.process = mp.Process(target= exec(self.script_codigo.get("1.0", self.tk.END))) # La especificación "1.0" indica que comienza desde la primera línea y el primer carácter, y tk.END indica que va hasta el final del contenido
                self.process.start()
                self.process.join()
            else:
                self.nodo.get_logger().warning("El anterior proceso sigue vivo, aun no se puede ejecutar uno nuevo")
        except Exception as e:
            self.nodo.get_logger().error(f"No se pudo ejecutar el codigo, se produjo la siguiente excepcion:\nTipo: {type(e).__name__}\nExcepcion: {e}")

    def abrir_archivo_python(self):
        ruta_archivo = filedialog.askopenfilename()
        if ruta_archivo: # para saber si ruta_archivo no esta vacia, es decir, que el usuario si selecciono un archivo
            if ruta_archivo.lower().endswith(".py"):
                self.script_codigo.delete("1.0", self.tk.END)
                with open(ruta_archivo, 'r', encoding= "utf-8") as archivo:
                    contenido = archivo.read()
                self.script_codigo.insert("1.0", contenido)
            else:
                nombre_archivo, extension = os.path.splitext(ruta_archivo)
                self.nodo.get_logger().warning(f"Debe elegir un archivo Python [.py], el que eligio es de extension [{extension}]")

    def guardar_archivo_python(self):
        codigo = self.script_codigo.get("1.0", self.tk.END)
        ruta_archivo = filedialog.asksaveasfilename(defaultextension=".py", filetypes=[("Archivos de Python", "*.py")]) # filestypes es una lista de tuplas que tienen el nombre del tipo de archivo y luego su extension
        nombre_archivo, extension = os.path.splitext(ruta_archivo)
        if ruta_archivo and extension == ".py":
            with open(ruta_archivo, "w") as archivo:
                archivo.write(codigo)
                self.nodo.get_logger().info("Archivo guardado exitosamente.")
        elif ruta_archivo:
            self.nodo.get_logger().warning(f"La extension del archivo es invalida, debe ser una extension de Python [.py] no [{extension}]")
        else:
            self.nodo.get_logger().warning("No se guardo el archivo")

    def crear_menu_archivos(self):
        menu = self.ttk.Menu(master= self.ventana)
        menu_archivo = self.ttk.Menu(master= menu, tearoff= 0) # tearoff es para decirle al sistema si puede crear un menu en otra ventana, el 0 significa que no, el 1 significa que si
        menu_archivo.add_command(label= "Abrir", command= self.abrir_archivo_python)
        menu_archivo.add_command(label= "Guardar", command= self.guardar_archivo_python)
        menu.add_cascade(label= "Archivo", menu= menu_archivo)
        self.ventana.config(menu= menu)

    def crear_cinematica_inversa(self, frame_contenedor):
        # Cinematica inversa
        # ventana.update_idletasks() # Actualiza la informacion sobre la ventana antes de usar las funciones que obtienen su ancho y alto
        script_frame = self.ttk.Frame(master= frame_contenedor)
        script_label1 = self.ttk.Label(master= script_frame, text= "Cinematica inversa", font= "TimesNewRoman 18")
        script_label2 = self.ttk.Label(master= script_frame, text= "Escriba el codigo python a ejecutar:", font= "TimesNewRoman 14")
        script_codigo = scrolledtext.ScrolledText(master= script_frame, wrap= self.tk.WORD, height= 10) # width y height esta en terminos de caracteres y lineas repectivamente, no pixeles
        self.script_codigo = script_codigo
        script_boton = self.ttk.Button(master= script_frame, text= "Ejecutar codigo", style= "TButton outline success", command= self.ejecutar_script)
        script_frame.pack(side= "top", padx= 20, pady= 10, fill= "x")
        script_label1.pack(side= "top", padx= 10, pady= 10, anchor= "w")
        script_label2.pack(side= "top", padx= 10, pady= 10, anchor= "w")
        script_codigo.pack(side= "top", fill= "x")
        script_boton.pack(side= "bottom", pady= 20)