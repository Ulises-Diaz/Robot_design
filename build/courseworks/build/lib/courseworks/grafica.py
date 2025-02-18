#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np
import matplotlib.pyplot as plt

class SignalPlotter(Node):
    def __init__(self):
        super().__init__('signal_plotter')
        
        # Inicializar listas para almacenar datos
        self.max_points = 100
        self.times = []
        self.signal_values = []
        self.proc_values = []
        self.current_time = 0.0
        
        # Suscriptores
        self.create_subscription(Float32, '/signal', self.signal_callback, 10)
        self.create_subscription(Float32, '/proc_signal', self.proc_signal_callback, 10)
        self.create_subscription(Float32, '/time', self.time_callback, 10)
        
        # Configurar la gráfica
        plt.ion()  # Modo interactivo
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8))
        self.fig.suptitle('Señales en Tiempo Real')

        # Configurar líneas
        self.line1, = self.ax1.plot([], [], 'b-', label='Señal Original')
        self.line2, = self.ax2.plot([], [], 'r-', label='Señal Procesada')

        # Configurar ejes
        for ax in [self.ax1, self.ax2]:
            ax.set_xlim(0, 10)
            ax.set_ylim(-1.5, 1.5)
            ax.grid(True)
            ax.legend()

        self.ax1.set_ylabel('Amplitud')
        self.ax2.set_xlabel('Tiempo (s)')
        self.ax2.set_ylabel('Amplitud')

        # Timer para actualizar la gráfica
        self.create_timer(0.1, self.update_plot)

    def time_callback(self, msg):
        self.current_time = msg.data

    def signal_callback(self, msg):
        """ Callback de la señal original. """
        self.times.append(self.current_time)
        self.signal_values.append(msg.data)

        # Mantener solo los últimos max_points valores
        if len(self.times) > self.max_points:
            self.times.pop(0)
            self.signal_values.pop(0)
            
        # Sincronizar proc_values con signal_values
        while len(self.proc_values) < len(self.signal_values):
            self.proc_values.append(0.0)

    def proc_signal_callback(self, msg):
        """ Callback de la señal procesada. """
        if len(self.proc_values) >= self.max_points:
            self.proc_values.pop(0)
        self.proc_values.append(msg.data)

    def update_plot(self):
        """ Actualiza la gráfica en tiempo real. """
        if len(self.times) > 1:
            # Actualizar datos
            self.line1.set_data(self.times, self.signal_values)
            self.line2.set_data(self.times[-len(self.proc_values):], self.proc_values)
            
            # Ajustar límites del eje x
            xmin = max(0, self.times[0])
            xmax = max(self.times)
            
            for ax in [self.ax1, self.ax2]:
                ax.set_xlim(xmin, xmax)
            
            # Redibujar
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    plotter = SignalPlotter()

    try:
        rclpy.spin(plotter)
    except KeyboardInterrupt:
        plt.close('all')
    finally:
        plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
