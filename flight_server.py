#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Services de decollage / atterrissage / arret d'urgence pour le Crazyflie.

Services offerts (std_srvs/Trigger, donc appelables a la main) :
  /takeoff    -> monte jusqu'a hover_height et reste en vol stationnaire
  /land       -> redescend doucement puis coupe les commandes
  /emergency  -> ARRET D'URGENCE : coupe tout immediatement

Tout est ecrit ici : on n'utilise aucun service de decollage, d'atterrissage
ou d'urgence deja fourni par Crazyswarm2. Le wrapper ne sert qu'a relayer nos
messages Hover vers le drone (et a publier /pose et le LaserScan).

Le noeud publie en continu un message Hover sur cmd_hover tant que le drone
vole. Des qu'on arrete de publier, le firmware coupe les moteurs via son
watchdog : c'est la base de notre arret d'urgence.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger

# Le type Hover vient du wrapper crazyflie_ros2_multiranger / Crazyswarm2.
# Champs : vx, vy, yaw_rate (ou yawrate), z_distance (ou zDistance).
from crazyflie_interfaces.msg import Hover

# Etats de la machine d'etats
IDLE = 'IDLE'
TAKEOFF = 'TAKEOFF'
HOVER = 'HOVER'
LANDING = 'LANDING'
EMERGENCY = 'EMERGENCY'


def set_hover(msg, vx, vy, yaw_rate, z):
    """Remplit un message Hover sans dependre du nom exact des champs."""
    msg.vx = float(vx)
    msg.vy = float(vy)
    if hasattr(msg, 'yaw_rate'):
        msg.yaw_rate = float(yaw_rate)
    else:
        msg.yawrate = float(yaw_rate)
    if hasattr(msg, 'z_distance'):
        msg.z_distance = float(z)
    else:
        msg.zDistance = float(z)
    return msg


class FlightServer(Node):

    def __init__(self):
        super().__init__('flight_server')

        # --- Parametres modifiables sans recompiler ---
        self.declare_parameter('cmd_topic', '/cmd_hover')
        self.declare_parameter('hover_height', 0.4)     # m, consigne de vol stationnaire
        self.declare_parameter('max_height', 1.20)      # m, plafond de securite
        self.declare_parameter('takeoff_speed', 0.25)   # m/s de montee
        self.declare_parameter('land_speed', 0.20)      # m/s de descente
        self.declare_parameter('min_height', 0.08)      # m, hauteur de coupure
        self.declare_parameter('rate', 20.0)            # Hz de publication
        self.declare_parameter('max_flight_time', 60.0) # s, atterrissage force

        cmd_topic = self.get_parameter('cmd_topic').value
        self.hover_height = self.get_parameter('hover_height').value
        self.max_height = self.get_parameter('max_height').value
        self.takeoff_speed = self.get_parameter('takeoff_speed').value
        self.land_speed = self.get_parameter('land_speed').value
        self.min_height = self.get_parameter('min_height').value
        self.rate = self.get_parameter('rate').value
        self.max_flight_time = self.get_parameter('max_flight_time').value

        self.publisher_ = self.create_publisher(Hover, cmd_topic, 10)

        # --- Services ---
        self.create_service(Trigger, 'takeoff', self.takeoff_callback)
        self.create_service(Trigger, 'land', self.land_callback)
        self.create_service(Trigger, 'emergency', self.emergency_callback)

        # --- Etat ---
        self.state = IDLE
        self.height = 0.0           # consigne de hauteur courante
        self.takeoff_time = None

        self.timer = self.create_timer(1.0 / self.rate, self.control_loop)
        self.get_logger().info(
            'Pret. Services : /takeoff  /land  /emergency   (topic %s, hauteur %.2f m)'
            % (cmd_topic, self.hover_height))

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------
    def takeoff_callback(self, request, response):
        if self.state == EMERGENCY:
            response.success = False
            response.message = 'Arret d urgence actif : relancer le noeud'
            return response
        if self.state in (TAKEOFF, HOVER):
            response.success = False
            response.message = 'Deja en vol'
            return response

        self.height = self.min_height
        self.takeoff_time = time.monotonic()
        self.state = TAKEOFF
        self.get_logger().info('DECOLLAGE vers %.2f m' % self.hover_height)
        response.success = True
        response.message = 'Decollage en cours'
        return response

    def land_callback(self, request, response):
        if self.state not in (TAKEOFF, HOVER):
            response.success = False
            response.message = 'Le drone n est pas en vol'
            return response

        self.state = LANDING
        self.get_logger().info('ATTERRISSAGE')
        response.success = True
        response.message = 'Atterrissage en cours'
        return response

    def emergency_callback(self, request, response):
        self.stop_now('demande de l operateur')
        response.success = True
        response.message = 'Moteurs coupes'
        return response

    # ------------------------------------------------------------------
    # Arret d'urgence
    # ------------------------------------------------------------------
    def stop_now(self, raison):
        """Arret d'urgence, ecrit entierement par nous.

        On cesse immediatement toute publication sur cmd_hover. Prive de
        consigne, le firmware du Crazyflie met les moteurs a zero via son
        watchdog (comptez jusqu'a environ 2 s dans le pire cas).

        L'etat EMERGENCY est verrouille : plus aucun decollage n'est possible
        tant que le noeud n'a pas ete relance. C'est volontaire, pour qu'un
        appui accidentel sur la touche de decollage ne redonne pas les gaz
        apres un incident."""
        if self.state == EMERGENCY:
            return
        self.state = EMERGENCY
        self.height = 0.0
        self.get_logger().error('*** ARRET D URGENCE (%s) ***' % raison)
        self.get_logger().error('Plus aucune commande envoyee. Moteurs coupes.')
        self.get_logger().error('Relancer le noeud pour pouvoir redecoller.')

    # ------------------------------------------------------------------
    # Boucle de controle : publie la consigne a `rate` Hz
    # ------------------------------------------------------------------
    def control_loop(self):
        if self.state in (IDLE, EMERGENCY):
            return  # on ne publie rien : moteurs coupes

        dt = 1.0 / self.rate

        # Securite : atterrissage force si le vol dure trop longtemps
        if self.takeoff_time is not None and self.state in (TAKEOFF, HOVER):
            if time.monotonic() - self.takeoff_time > self.max_flight_time:
                self.get_logger().warn('Duree maximale atteinte : atterrissage')
                self.state = LANDING

        # La consigne de hauteur est relue a chaque cycle : on peut donc la
        # changer en vol avec  ros2 param set /flight_server hover_height 0.5
        cible = self.get_parameter('hover_height').value
        cible = max(self.min_height, min(cible, self.max_height))  # plafond de securite

        if self.state == TAKEOFF:
            self.height = min(self.height + self.takeoff_speed * dt, cible)
            if math.isclose(self.height, cible, abs_tol=1e-3):
                self.state = HOVER
                self.get_logger().info('Vol stationnaire a %.2f m' % self.height)

        elif self.state == HOVER:
            # Suivi doux si l'operateur change la hauteur demandee
            if self.height < cible - 1e-3:
                self.height = min(self.height + self.takeoff_speed * dt, cible)
            elif self.height > cible + 1e-3:
                self.height = max(self.height - self.land_speed * dt, cible)

        elif self.state == LANDING:
            self.height -= self.land_speed * dt
            if self.height <= self.min_height:
                self.height = 0.0
                self.state = IDLE
                self.takeoff_time = None
                self.get_logger().info('Pose. Moteurs coupes.')
                return  # on arrete de publier -> moteurs coupes

        msg = set_hover(Hover(), 0.0, 0.0, 0.0, self.height)
        if hasattr(msg, 'header'):
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'world'
        self.publisher_.publish(msg)


def main(args=None):
    # NO : on gere nous-memes le Ctrl+C pour couper proprement
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = FlightServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_now('Ctrl+C')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
