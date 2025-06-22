import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import threading
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('gui_subscriber')

        self.temps = []
        self.temp_values = []
        self.hum_values = []
        self.pres_values = []

        self.temp_range = (15, 35)
        self.hum_range = (30, 70)
        self.pres_range = (950, 1050)

        self.counter = 0
        self.lock = threading.Lock()

        self.create_subscription(Float32, 'sensor_data/temperature', self.temp_callback, 10)
        self.create_subscription(Float32, 'sensor_data/humidity', self.hum_callback, 10)
        self.create_subscription(Float32, 'sensor_data/pressure', self.pres_callback, 10)

        self.last_temp = None
        self.last_hum = None
        self.last_pres = None

    def temp_callback(self, msg):
        with self.lock:
            self.last_temp = msg.data
            self.update_data_if_ready()

    def hum_callback(self, msg):
        with self.lock:
            self.last_hum = msg.data
            self.update_data_if_ready()

    def pres_callback(self, msg):
        with self.lock:
            self.last_pres = msg.data
            self.update_data_if_ready()

    def update_data_if_ready(self):
        if self.last_temp is not None and self.last_hum is not None and self.last_pres is not None:
            self.counter += 1
            self.temps.append(self.counter)
            self.temp_values.append(self.last_temp)
            self.hum_values.append(self.last_hum)
            self.pres_values.append(self.last_pres)
            self.last_temp = self.last_hum = self.last_pres = None

    def get_data(self):
        with self.lock:
            return (list(self.temps), list(self.temp_values), list(self.hum_values), list(self.pres_values))


def ros_spin(node):
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SensorSubscriber()
    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    # Crée 4 sous-graphes (3 pour courbes, 1 pour tableau)
    fig, axs = plt.subplots(4, 1, figsize=(9, 12))
    ax1, ax2, ax3, table_ax = axs
    plt.subplots_adjust(hspace=0.6, bottom=0.2)

    def draw_band(ax, low, high, color='green', alpha=0.2):
        ax.axhspan(low, high, facecolor=color, alpha=alpha)

    def compute_stats(values, value_range):
        total = len(values)
        if total == 0:
            return (0, 0)
        good = sum(value_range[0] <= v <= value_range[1] for v in values)
        bad = total - good
        return (good / total * 100, bad / total * 100)

    def update(frame):
        temps, temp_vals, hum_vals, pres_vals = node.get_data()

        # Clear all 4 axes
        for ax in axs:
            ax.clear()

        if temps:
            # Température
            ymin, ymax = node.temp_range[0] - 5, node.temp_range[1] + 5
            ax1.set_ylim(ymin, ymax)
            draw_band(ax1, *node.temp_range, color='green')
            ax1.plot(temps, temp_vals, label='Température', color='blue')
            ax1.set_ylabel('°C')
            ax1.set_title('Température')
            ax1.legend()
            color_temp = 'green' if node.temp_range[0] <= temp_vals[-1] <= node.temp_range[1] else 'red'
            ax1.text(0.5, -0.35, f"Valeur actuelle : {temp_vals[-1]:.2f} °C", color=color_temp,
                     transform=ax1.transAxes, ha='center', fontsize=10)

            # Humidité
            ymin, ymax = node.hum_range[0] - 10, node.hum_range[1] + 10
            ax2.set_ylim(ymin, ymax)
            draw_band(ax2, *node.hum_range, color='green')
            ax2.plot(temps, hum_vals, label='Humidité', color='orange')
            ax2.set_ylabel('%')
            ax2.set_title('Humidité')
            ax2.legend()
            color_hum = 'green' if node.hum_range[0] <= hum_vals[-1] <= node.hum_range[1] else 'red'
            ax2.text(0.5, -0.35, f"Valeur actuelle : {hum_vals[-1]:.2f} %", color=color_hum,
                     transform=ax2.transAxes, ha='center', fontsize=10)

            # Pression
            ymin, ymax = node.pres_range[0] - 20, node.pres_range[1] + 20
            ax3.set_ylim(ymin, ymax)
            draw_band(ax3, *node.pres_range, color='green')
            ax3.plot(temps, pres_vals, label='Pression', color='purple')
            ax3.set_ylabel('hPa')
            ax3.set_title('Pression')
            ax3.legend()
            color_pres = 'green' if node.pres_range[0] <= pres_vals[-1] <= node.pres_range[1] else 'red'
            ax3.text(0.5, -0.35, f"Valeur actuelle : {pres_vals[-1]:.2f} hPa", color=color_pres,
                     transform=ax3.transAxes, ha='center', fontsize=10)

            # Tableau de statistiques
            temp_good, temp_bad = compute_stats(temp_vals, node.temp_range)
            hum_good, hum_bad = compute_stats(hum_vals, node.hum_range)
            pres_good, pres_bad = compute_stats(pres_vals, node.pres_range)

            table_data = [
                ["Température", len(temp_vals), f"{temp_good:.1f}%", f"{temp_bad:.1f}%"],
                ["Humidité", len(hum_vals), f"{hum_good:.1f}%", f"{hum_bad:.1f}%"],
                ["Pression", len(pres_vals), f"{pres_good:.1f}%", f"{pres_bad:.1f}%"]
            ]
            col_labels = ["Mesure", "Total", "Bonnes", "Mauvaises"]

            table_ax.axis("off")
            table = table_ax.table(cellText=table_data, colLabels=col_labels, loc='center', cellLoc='center')

            for i in range(1, 4):
                table[(i, 2)].set_facecolor("#d0f0c0")  # vert clair
                table[(i, 3)].set_facecolor("#f8d0d0")  # rouge clair

            table.scale(1, 1.5)
            table.set_fontsize(10)
            table_ax.set_title("Bilan Statistique des Mesures", fontsize=12, fontweight="bold", pad=10)
        else:
            for ax in axs:
                ax.text(0.5, 0.5, "Aucune donnée reçue", ha='center', va='center', transform=ax.transAxes)

    ani = FuncAnimation(fig, update, interval=1000)
    plt.show()
    ros_thread.join()


if __name__ == '__main__':
    main()
