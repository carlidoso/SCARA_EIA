import os
from PIL import Image, ImageTk

class CinematicaDirecta():
    '''
    Estructura de la aplicacion que sera usada para la cinematica directa.

    Contiene las siguientes funciones:
    * enviar_cinematica_directa: Para enviar al ESP32 el angulo escogido. No tiene parametros
    * sumar_angulo: Se encarga de sumar el valor que halla en la lista desplegable al valor que se vaya a enviar. Parametros:
        1. cine_dire_entry: Objeto de tipo 'entry', contiene el valor que vamos a enviar.
        2. cine_dire_entry_double: Objeto que es el valor double que vamos a enviar.
        3. cine_dire_combo: Objeto de la lista desplegable.
    * restar_angulo: Se encarga de restar el valor que halla en la lista desplegable al valor que se vaya a enviar. Parametros:
        1. cine_dire_entry: Objeto de tipo 'entry', contiene el valor que vamos a enviar.
        2. cine_dire_entry_double: Objeto que es el valor double que vamos a enviar.
        3. cine_dire_combo: Objeto de la lista desplegable.
    * crear_cinematica_directa: Funcion mas importante, se encarga de crear todas las partes de la cinematica directa. No tiene parametros.
    * servomotor: Funcion utilizada para crear cada seccion de un servomotor. Parametros:
        1. icono_mas: Archivo de imagen para el boton de sumar.
        2. icono_menos: Archivo de imagen para el boton de restar.
        3. opciones_desplegable: Array que contiene cuales son las opciones de la lista desplegable.
        4. numero_servo: Es un entero para diferenciar un servomotor de otro.
    '''
    def __init__(self, tk, ttk, frame_contenedor, ancho_pantalla, req_motor_ang1_publisher, req_motor_ang2_publisher, tipo_vector, nodo) -> None:
        # Rutas de imagenes
        home_dir = os.path.expanduser('~')
        directorio_actual = home_dir+"/Nexos/ros2_ws/src/nexos_ros2/resource/app_scara/images"
        self.ruta_signo_mas = os.path.join(directorio_actual, "signo_mas.png")
        self.ruta_signo_menos = os.path.join(directorio_actual, "signo_menos.png")
        self.tk = tk
        self.ttk = ttk
        self.servo1_ang = 0.0
        self.servo2_ang = 0.0
        self.servo3_ang = 0.0
        self.servo4_cm = 25.4
        self.reiniciar = 0.0
        self.req_motor_ang1_publisher = req_motor_ang1_publisher
        self.req_motor_ang2_publisher = req_motor_ang2_publisher
        self.frame_contenedor = frame_contenedor
        self.ancho_pantalla = ancho_pantalla
        self.tipo_vector = tipo_vector
        self.nodo = nodo
        self.crear_cinematica_directa()

    def enviar_cinematica_directa(self, servo, msg):
        try:
            msg_ = self.tipo_vector()
            if servo == 1 and -62 <= msg <= 62:
                self.servo1_ang = msg
                msg_.x = self.servo1_ang
                msg_.y = self.servo2_ang
                msg_.z = self.servo3_ang
                self.req_motor_ang1_publisher.publish(msg_)
            elif servo == 2 and -105 <= msg <= 105:
                self.servo2_ang = msg
                msg_.x = self.servo1_ang
                msg_.y = self.servo2_ang
                msg_.z = self.servo3_ang
                self.req_motor_ang1_publisher.publish(msg_)
            elif servo == 3 and 0 <= msg <= 360:
                self.servo3_ang = msg
                msg_.x = self.servo1_ang
                msg_.y = self.servo2_ang
                msg_.z = self.servo3_ang
                self.req_motor_ang1_publisher.publish(msg_)
            elif servo == 4 and 0 <= msg <= 25.4:
                self.servo4_cm = msg
                msg_.x = self.servo4_cm
                msg_.y = self.reiniciar
                self.req_motor_ang2_publisher.publish(msg_)
            else: 
                self.nodo.get_logger().warning("No se puede enviar el valor solicitado, fuera de rango")
        except Exception as e:
            self.nodo.get_logger().error(f"Se presento un error en enviar_cinematica_directa: {e}")

    def sumar_angulo(self, cine_dire_entry, cine_dire_entry_double, cine_dire_combo, servo):
        try:
            valor_actual = cine_dire_entry_double.get()
            valor_nuevo = valor_actual + float(cine_dire_combo.get())
            if servo == 1 and -62 <= valor_nuevo <= 62:
                cine_dire_entry.delete(0, self.tk.END)
                cine_dire_entry.insert(0, valor_nuevo)
                self.enviar_cinematica_directa(servo, valor_nuevo)
            elif servo == 2 and -105 <= valor_nuevo <= 105: 
                cine_dire_entry.delete(0, self.tk.END)
                cine_dire_entry.insert(0, valor_nuevo)
                self.enviar_cinematica_directa(servo, valor_nuevo)
            elif servo == 3 and 0 <= valor_nuevo <= 360: 
                cine_dire_entry.delete(0, self.tk.END)
                cine_dire_entry.insert(0, valor_nuevo)
                self.enviar_cinematica_directa(servo, valor_nuevo)
            elif servo == 4 and 0 <= valor_nuevo <= 25.4: 
                cine_dire_entry.delete(0, self.tk.END)
                cine_dire_entry.insert(0, valor_nuevo)
                self.enviar_cinematica_directa(servo, valor_nuevo)
            else:
                self.nodo.get_logger().warning("No se puede enviar el valor solicitado, fuera de rango")
        except self.tk._tkinter.TclError as e:
            self.nodo.get_logger().warning(f"Debe escribir un numero valido, el valor de entrada fue: {e}")
        except Exception as e:
            self.nodo.get_logger().error(f"Se presento un error en sumar_angulo: {e}")

    def restar_angulo(self, cine_dire_entry, cine_dire_entry_double, cine_dire_combo, servo):
        try:
            valor_actual = cine_dire_entry_double.get()
            valor_nuevo = valor_actual - float(cine_dire_combo.get())
            if servo == 1 and -62 <= valor_nuevo <= 62:
                cine_dire_entry.delete(0, self.tk.END)
                cine_dire_entry.insert(0, valor_nuevo)
                self.enviar_cinematica_directa(servo, valor_nuevo)
            elif servo == 2 and -105 <= valor_nuevo <= 105: 
                cine_dire_entry.delete(0, self.tk.END)
                cine_dire_entry.insert(0, valor_nuevo)
                self.enviar_cinematica_directa(servo, valor_nuevo)
            elif servo == 3 and 0 <= valor_nuevo <= 360: 
                cine_dire_entry.delete(0, self.tk.END)
                cine_dire_entry.insert(0, valor_nuevo)
                self.enviar_cinematica_directa(servo, valor_nuevo)
            elif servo == 4 and 0 <= valor_nuevo <= 25.4: 
                cine_dire_entry.delete(0, self.tk.END)
                cine_dire_entry.insert(0, valor_nuevo)
                self.enviar_cinematica_directa(servo, valor_nuevo)
            else:
                self.nodo.get_logger().warning("No se puede enviar el valor solicitado, fuera de rango")

        except self.tk._tkinter.TclError as e:
            self.nodo.get_logger().warning(f"Debe escribir un numero valido, el valor de entrada fue: {e}")
        except Exception as e:
            self.nodo.get_logger().error(f"Se presento un error en restar_angulo: {e}")
            
    def reiniciar_posicion(self):
        try:
            msg_ = self.tipo_vector()
            self.reiniciar = 1.0
            msg_.x = self.servo4_cm
            msg_.y = self.reiniciar
            self.req_motor_ang2_publisher.publish(msg_)
            self.reiniciar = 0.0
        except Exception as e:
            self.nodo.get_logger().error(f"Se presento un error en reiniciar_posicion: {e}")
            
    def home(self):
        try:
            msg_ = self.tipo_vector()
            msg_.x = self.servo4_cm
            msg_.y = self.reiniciar
            msg_.z = 1.0
            self.req_motor_ang2_publisher.publish(msg_)
        except Exception as e:
            self.nodo.get_logger().error(f"Se presento un error en home: {e}")

    def crear_cinematica_directa(self):
        opciones_desplegable = [1, 45, 90, 180]
        opciones_desplegable_cm = [1, 2, 5, 10]
        label_principal = self.ttk.Label(master= self.frame_contenedor, text= "Cinematica directa", font= "TimesNewRoman 18")
        boton_reinicio = self.ttk.Button(master= self.frame_contenedor, text= "Posicionar", style= "TButton outline info", command= self.reiniciar_posicion)
        boton_home = self.ttk.Button(master= self.frame_contenedor, text= "Home", style= "TButton outline info", command= self.home)
        label_principal.pack(side= "top", padx= 30, pady= 10, anchor= "w")
        boton_home.pack(side= "top", padx= 30, pady= 10, anchor= "w")
        boton_reinicio.pack(side= "top", padx= 30, pady= 10, anchor= "w")
        imagen_mas = Image.open(self.ruta_signo_mas)
        imagen_mas = imagen_mas.resize((20, 20), Image.LANCZOS)  # Ajusta el tamaño de la imagen, LANCZOS es para evitar el aliasing, es decir, distorsiones en la imagen
        icono_mas = ImageTk.PhotoImage(image= imagen_mas) # Aqui lo preparamos para que tkinter lo pueda usar
        imagen_menos = Image.open(self.ruta_signo_menos)
        imagen_menos = imagen_menos.resize((20, 20), Image.LANCZOS)  # Ajusta el tamaño de la imagen, LANCZOS es para evitar el aliasing, es decir, distorsiones en la imagen
        icono_menos = ImageTk.PhotoImage(image= imagen_menos) # Aqui lo preparamos para que tkinter lo pueda usar
        ## Servomotor 1
        self.servomotor(icono_mas, icono_menos, opciones_desplegable, 1, self.servo1_ang)
        ## Servomotor 2
        self.servomotor(icono_mas, icono_menos, opciones_desplegable, 2, self.servo2_ang)
        ## Servomotor 3
        self.servomotor(icono_mas, icono_menos, opciones_desplegable, 3, self.servo3_ang)
        ## Servomotor 4
        self.servomotor(icono_mas, icono_menos, opciones_desplegable_cm, 4, self.servo4_cm)

    def servomotor(self, icono_mas, icono_menos, opciones_desplegable, numero_servo, double_inicial):
        frame1_servo = self.ttk.Frame(master= self.frame_contenedor)
        frame2_servo = self.ttk.Frame(master= self.frame_contenedor)
        label_servo = self.ttk.Label(master= frame1_servo, text= f"Servomotor {numero_servo}", font= "TimesNewRoman 15")
        enviar_servo = self.ttk.Button(master= frame1_servo, text= "Enviar", style= "TButton outline info", command= lambda: self.enviar_cinematica_directa(numero_servo, double_servo.get()))
        sumar_servo = self.ttk.Button(master= frame2_servo, image= icono_mas, style= "TButton outline info", command= lambda: self.sumar_angulo(entry_servo, double_servo, combo_servo, numero_servo))
        sumar_servo.image = icono_mas # Para evitar el garbage collector de python
        restar_servo = self.ttk.Button(master= frame2_servo, image= icono_menos, style= "TButton outline info", command= lambda: self.restar_angulo(entry_servo, double_servo, combo_servo, numero_servo))
        restar_servo.image = icono_menos # Para evitar el garbage collector de python
        double_servo = self.tk.DoubleVar(value= double_inicial)
        entry_servo = self.ttk.Entry(master= frame1_servo, textvariable= double_servo)
        combo_servo = self.ttk.Combobox(master= frame2_servo, values= opciones_desplegable, width= 4, style= "info")
        combo_servo.set(opciones_desplegable[0])
        frame1_servo.pack(side= "top", pady= 10, anchor= 'w')#
        label_servo.pack(side= "top", padx= 30, pady= 20, anchor= "w")
        entry_servo.pack(side= "left", padx= 30)
        enviar_servo.pack(side="left", padx= 10)
        frame2_servo.pack(side= "top", pady= 10, anchor= 'w')#
        combo_servo.pack(side= "left", padx= 30)
        sumar_servo.pack(side= "left", padx= 10)
        restar_servo.pack(side= "left", padx= 10)

if __name__ == "__main__":
    pass