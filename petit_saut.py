"""
PETIT SAUT : decolle de quelques cm, tient 2 s, atterrit.

Le controle d'altitude est fait ICI, sur le PC, de zero :
  - on lit directement le laser du Flow deck (range.zrange)
  - notre propre PID calcule la poussee
  - on envoie au drone seulement : roll=0, pitch=0, yawrate=0, thrust
Pas de high-level commander, pas de MotionCommander, pas de Kalman,
pas de controle de position du Crazyflie.
(Seule la stabilisation d'attitude embarquee reste : sans elle, rien ne vole.)

Tout est enregistre dans logs/vol_AAAAMMJJ_HHMMSS.csv
Ensuite : python3 analyse_vol.py  -> graphiques + diagnostic automatique

Ctrl+C a tout moment = moteurs coupes.
"""

import csv
import math
import os
import time
from datetime import datetime

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig

# ======================= REGLAGES =======================
URI = "radio://0/80/2M/E7E7E7E7E2"

HAUTEUR = 0.15          # m au-dessus du sol
DUREE_MONTEE = 1.0      # s
DUREE_HOVER = 30.0     # s
DUREE_DESCENTE = 2.0    # s

# Recherche de la poussee de decollage : on monte doucement la poussee
# jusqu'a ce que le laser voie le drone quitter le sol.
# Montee RAPIDE : si le drone reste longtemps au sol avec les moteurs qui tournent,
# le PID d'attitude embarque accumule de la correction et le drone fait un bond au decollage.
THRUST_DEPART = 46000
RAMPE_THRUST = 20000    # unites de thrust par seconde
THRUST_MAX = 60000      # jamais plus (65535 = max absolu, on garde de la marge pour l'attitude)
THRUST_MIN_VOL = 20000  # jamais moins en vol (evite de couper en l'air)
SEUIL_DECOLLAGE = 0.015 # m : au-dessus = le drone a quitte le sol

# Notre PID d'altitude (sortie en unites de thrust). Gains volontairement doux.
KP = 20000              # par metre d'erreur
KI = 10000              # par metre * seconde
KD = 15000              # par m/s de vitesse
I_MAX = 6000            # limite de l'effet integral

# Securites -> moteurs coupes immediatement
ANGLE_MAX = 25          # deg de roll ou pitch

# TRIM = True : on demande au drone de garder l'angle mesure au sol au lieu de 0.
# Desactive : le vol du 17/11 12:20 a montre que l'angle de ~3-4 deg au sol etait
# une erreur de l'estimateur (il est retombe a 0 tout seul), et le trim a fait pencher le drone.
# Maintenant on remet l'estimateur a zero avant de mesurer, donc plus besoin de trim.
TRIM = False

# --- Notre controle de position horizontale (pour qu'il reste sur place) ---
# On lit la position/vitesse estimees a partir du Flow deck, et c'est NOTRE code
# qui decide de combien pencher le drone pour revenir au point de depart.
CONTROLE_XY = True
KP_XY = 20.0            # deg d'inclinaison par metre d'ecart
KD_XY = 15.0            # deg d'inclinaison par m/s de vitesse
ANGLE_CMD_MAX = 8.0     # deg max demandes par le controle XY
DERIVE_MAX = 0.5        # m : au-dela, on ARRETE le controle XY (sans couper les moteurs)
VITESSE_XY_MAX = 1.0    # m/s : vitesse mesuree au-dela = mesure douteuse -> controle XY arrete
SAUT_XY_MAX = 0.20      # m : saut de position en 20 ms au-dela = mesure douteuse
Z_XY_MIN = 0.08         # m : la camera du Flow deck ne voit rien d'utile en dessous.
                        # Le point de reference est pris quand le drone passe cette hauteur.
ANGLE_SOL_MAX = 8       # deg : au-dela, on refuse de decoller
Z_MAX = 0.30            # m
TIMEOUT_CAPTEUR = 0.3   # s sans donnees du drone
TEMPS_MAX = 45          # s au total
VBAT_MIN_REPOS = 2.0    # V avant de decoller

PERIODE = 0.02          # boucle a 50 Hz
# ========================================================

d = {}  # dernieres valeurs recues du drone
etat = {"z": None, "vz": 0.0, "ts_chg": 0, "t_recu": None}

COLONNES = ["t", "phase", "z_cible", "z", "vz", "thrust", "erreur", "P", "I", "D",
            "thrust_recu", "roll", "pitch", "yaw", "gyro_x", "gyro_y", "gyro_z", "acc_z",
            "m1", "m2", "m3", "m4", "vbat", "squal", "supervisor",
            "att_roll_I", "att_pitch_I", "rate_roll_I", "rate_pitch_I",
            "x", "y", "vx", "vy", "cmd_roll", "cmd_pitch", "evenement"]


# ---------- Reception des donnees ----------
def cb_altitude(ts, data, conf):
    """Bloc principal : laser + attitude. Calcule aussi la vitesse verticale."""
    d.update(data)
    z = data["range.zrange"] / 1000.0
    if etat["z"] is None:
        etat["z"], etat["ts_chg"] = z, ts
    elif z != etat["z"]:
        dt = max((ts - etat["ts_chg"]) / 1000.0, 0.005)
        v = (z - etat["z"]) / dt
        etat["vz"] = 0.6 * etat["vz"] + 0.4 * v      # petit filtre
        etat["z"], etat["ts_chg"] = z, ts
    elif ts - etat["ts_chg"] > 100:
        etat["vz"] = 0.0
    etat["t_recu"] = time.perf_counter()


def cb_simple(ts, data, conf):
    d.update(data)


def ajouter_bloc(cf, nom, periode_ms, variables, cb):
    conf = LogConfig(name=nom, period_in_ms=periode_ms)
    n = 0
    for v, t in variables:
        if cf.log.toc.get_element_by_complete_name(v) is not None:
            conf.add_variable(v, t)
            n += 1
        else:
            print(f"  (variable absente dans ce firmware : {v})")
    if n == 0:
        return None
    cf.log.add_config(conf)
    conf.data_received_cb.add_callback(cb)
    conf.start()
    return conf


def set_param(cf, nom, valeur, timeout=3.0):
    """Ecrit un parametre sans jamais bloquer le script plus de `timeout` s."""
    import threading
    th = threading.Thread(target=cf.param.set_value, args=(nom, valeur), daemon=True)
    th.start()
    th.join(timeout)
    if th.is_alive():
        print(f"  [!] le drone ne repond pas pour {nom} (on continue sans)", flush=True)
        return False
    return True


def etat_superviseur():
    """Traduit supervisor.info (etat de securite du firmware) en texte."""
    info = d.get("supervisor.info")
    if info is None:
        return None, ""
    info = int(info)
    noms = {0x02: "arme", 0x08: "peut voler", 0x10: "en vol", 0x20: "RETOURNE",
            0x40: "VERROUILLE", 0x80: "CRASH DETECTE"}
    return info, ", ".join(n for b, n in noms.items() if info & b) or "rien"


# ---------- Ecriture du log ----------
def r(cle, n=2):
    v = d.get(cle, "")
    return round(v, n) if isinstance(v, float) else v


cmd = {"roll": 0.0, "pitch": 0.0}   # derniere consigne d'angle envoyee (pour le log)


def ecrire(w, t, phase, z_cible, z, thrust, e, P, I, D, evt=""):
    w.writerow([round(t, 3), phase, round(z_cible, 4), round(z, 4), round(etat["vz"], 4),
                int(thrust), round(e, 4), int(P), int(I), int(D),
                r("stabilizer.thrust", 0), r("stabilizer.roll"), r("stabilizer.pitch"), r("stateEstimate.yaw"),
                r("gyro.x"), r("gyro.y"), r("gyro.z"), r("acc.z", 3),
                r("motor.m1"), r("motor.m2"), r("motor.m3"), r("motor.m4"),
                r("pm.vbat", 3), r("motion.squal"), r("supervisor.info"),
                r("pid_attitude.roll_outI"), r("pid_attitude.pitch_outI"),
                r("pid_rate.roll_outI"), r("pid_rate.pitch_outI"),
                r("stateEstimate.x", 3), r("stateEstimate.y", 3),
                r("stateEstimate.vx", 3), r("stateEstimate.vy", 3),
                round(cmd["roll"], 2), round(cmd["pitch"], 2), evt])


# ---------- Le vol ----------
def voler(cf, w):
    # --- Verifs au sol ---
    if etat["z"] is None:
        return "Aucune donnee du laser recue"
    vbat = d.get("pm.vbat", 0.0)
    print(f"Batterie : {vbat:.2f} V")
    if vbat < VBAT_MIN_REPOS:
        return f"Batterie trop faible au repos ({vbat:.2f} V < {VBAT_MIN_REPOS} V)"
    # Si le superviseur s'est verrouille (crash detecte au vol precedent), on le debloque
    if hasattr(cf, "supervisor") and hasattr(cf.supervisor, "send_crash_recovery_request"):
        cf.supervisor.send_crash_recovery_request()
        time.sleep(0.5)
    print(f"Etat superviseur (supervisor.info) : {d.get('supervisor.info', '?')}")

    # Armement
    if hasattr(cf, "supervisor") and hasattr(cf.supervisor, "send_arming_request"):
        cf.supervisor.send_arming_request(True)
        time.sleep(1.0)
    elif hasattr(cf, "platform") and hasattr(cf.platform, "send_arming_request"):
        cf.platform.send_arming_request(True)
        time.sleep(1.0)
    print("Mesure au sol (1 s, ne touche pas le drone)...", flush=True)

    mesures, rolls, pitchs = [], [], []
    for _ in range(50):
        mesures.append(etat["z"])
        rolls.append(d.get("stabilizer.roll", 0.0))
        pitchs.append(d.get("stabilizer.pitch", 0.0))
        time.sleep(0.02)
    z0 = sum(mesures) / len(mesures)
    roll0 = sum(rolls) / len(rolls)
    pitch0 = sum(pitchs) / len(pitchs)
    print(f"Hauteur du laser au sol : {z0 * 1000:.0f} mm (sert de zero)")
    print(f"Angle mesure au sol : roll={roll0:.1f}  pitch={pitch0:.1f} deg")
    if abs(roll0) > ANGLE_SOL_MAX or abs(pitch0) > ANGLE_SOL_MAX:
        return f"Drone pas a plat (roll={roll0:.1f}, pitch={pitch0:.1f})"

    # Consigne d'angle envoyee. Attention : cflib inverse le pitch dans send_setpoint,
    # donc pour garder le pitch mesure p il faut envoyer -p.
    if TRIM:
        trim_roll, trim_pitch = roll0, -pitch0
        print(f"Trim actif : consigne roll={trim_roll:.1f}  pitch={trim_pitch:.1f}")
    else:
        trim_roll, trim_pitch = 0.0, 0.0
    x0 = y0 = None   # point de reference, pris en l'air (pas au sol : l'estimation y derive)
    xy_actif = CONTROLE_XY
    x_prec = y_prec = None
    if CONTROLE_XY and "stateEstimate.x" not in d:
        print("  [!] pas de position estimee recue : controle XY desactive")

    info, txt = etat_superviseur()
    if info is not None:
        print(f"Etat securite du drone : {txt}")
        if info & (0x40 | 0x80):
            return ("Le drone est VERROUILLE par sa securite (crash detecte au vol precedent). "
                    "Eteins-le et rallume-le.")

    # Poussee 0 pendant 0.3 s : deverrouille + remet a zero les PID embarques
    for _ in range(15):
        cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.02)

    # --- Boucle de vol ---
    t0 = time.perf_counter()
    t_phase = t0
    phase = "recherche"
    thrust = THRUST_DEPART
    base = 0.0
    integ = 0.0
    z_cible = 0.0
    z_depart = 0.0
    n_haut = 0
    evt = "Debut recherche poussee"
    print("Recherche de la poussee de decollage...")

    while True:
        tb = time.perf_counter()
        t, tp = tb - t0, tb - t_phase
        z = etat["z"] - z0
        vz = etat["vz"]
        roll = d.get("stabilizer.roll", 0.0)
        pitch = d.get("stabilizer.pitch", 0.0)
        e = P = I = D = 0.0

        # --- Securites ---
        raison = None
        if abs(roll) > ANGLE_MAX or abs(pitch) > ANGLE_MAX:
            raison = f"Angle trop grand (roll={roll:.0f}, pitch={pitch:.0f} deg)"
        elif z > Z_MAX:
            raison = f"Trop haut ({z * 100:.0f} cm)"
        elif tb - etat["t_recu"] > TIMEOUT_CAPTEUR:
            raison = "Plus de donnees du drone depuis 0.3 s (radio ?)"
        elif t > TEMPS_MAX:
            raison = "Temps max depasse"

        if raison:
            cf.commander.send_stop_setpoint()
            ecrire(w, t, phase, z_cible, z, 0, e, P, I, D, "ARRET: " + raison)
            return "ARRET D'URGENCE : " + raison

        # --- Phases ---
        if phase == "recherche":
            # Le drone applique-t-il vraiment la poussee ? (sinon il est verrouille)
            if tp > 0.7 and d.get("stabilizer.thrust", 1) < 1000:
                cf.commander.send_stop_setpoint()
                _, txt = etat_superviseur()
                ecrire(w, t, phase, 0, z, 0, 0, 0, 0, 0, "ARRET: poussee ignoree par le drone")
                return ("Le drone IGNORE la poussee (moteurs a 0). Securite du firmware : "
                        f"[{txt}]. Eteins et rallume le drone.")
            thrust = THRUST_DEPART + RAMPE_THRUST * tp
            n_haut = n_haut + 1 if z > SEUIL_DECOLLAGE else 0
            if n_haut >= 3:
                # le laser voit le decollage avec ~0.2 s de retard : on retire ce qui a ete ajoute entre-temps
                base = thrust - RAMPE_THRUST * 0.2 - 1000
                z_depart = z
                evt = f"Decollage detecte a thrust={int(thrust)}"
                phase, t_phase = "montee", tb
            elif thrust > THRUST_MAX and tp > (THRUST_MAX - THRUST_DEPART) / RAMPE_THRUST + 0.5:
                # on a tenu la poussee max 0.5 s sans decoller
                cf.commander.send_stop_setpoint()
                ecrire(w, t, phase, 0, z, 0, 0, 0, 0, 0, "ARRET: pas de decollage")
                return (f"Pas de decollage meme a thrust={THRUST_MAX} "
                        "(trop lourd, batterie faible ou moteur faible)")
        else:
            vz_cible = 0.0
            if phase == "montee":
                k = min(tp / DUREE_MONTEE, 1.0)
                z_cible = z_depart + (HAUTEUR - z_depart) * k
                vz_cible = (HAUTEUR - z_depart) / DUREE_MONTEE if k < 1 else 0.0
                if tp >= DUREE_MONTEE:
                    phase, t_phase, evt = "hover", tb, "Debut hover"
            elif phase == "hover":
                z_cible = HAUTEUR
                if tp >= DUREE_HOVER:
                    phase, t_phase, evt = "descente", tb, "Debut descente"
            elif phase == "descente":
                k = min(tp / DUREE_DESCENTE, 1.0)
                z_cible = HAUTEUR * (1 - k)
                vz_cible = -HAUTEUR / DUREE_DESCENTE if k < 1 else 0.0
                if (k >= 1 and z < SEUIL_DECOLLAGE) or tp > DUREE_DESCENTE + 1.5:
                    cf.commander.send_stop_setpoint()
                    ecrire(w, t, phase, z_cible, z, 0, 0, 0, 0, 0, "Pose, moteurs coupes")
                    return "OK : decollage et atterrissage reussis"

            # --- Notre PID ---
            e = z_cible - z
            integ += e * PERIODE
            integ = max(-I_MAX / KI, min(I_MAX / KI, integ))
            P = KP * e
            I = KI * integ
            D = KD * (vz_cible - vz)
            thrust = base + P + I + D
            thrust = max(THRUST_MIN_VOL, thrust)

        thrust = min(THRUST_MAX, thrust)

        # --- Notre controle XY (seulement une fois en l'air) ---
        cmd_roll, cmd_pitch = trim_roll, trim_pitch
        if (xy_actif and x0 is None and phase in ("montee", "hover")
                and z > Z_XY_MIN and "stateEstimate.x" in d):
            x0, y0 = d["stateEstimate.x"], d["stateEstimate.y"]
            evt = (evt + " | " if evt else "") + f"Controle XY actif (reference prise a {z * 100:.0f} cm)"
        if xy_actif and x0 is not None:
            # ecart et vitesse dans le repere du monde...
            ex = d["stateEstimate.x"] - x0
            ey = d["stateEstimate.y"] - y0
            vx, vy = d.get("stateEstimate.vx", 0.0), d.get("stateEstimate.vy", 0.0)
            # Mesure douteuse ? -> on arrete le controle XY mais on continue a voler
            saut = 0.0 if x_prec is None else math.hypot(d["stateEstimate.x"] - x_prec,
                                                         d["stateEstimate.y"] - y_prec)
            x_prec, y_prec = d["stateEstimate.x"], d["stateEstimate.y"]
            douteux = None
            if math.hypot(ex, ey) > DERIVE_MAX:
                douteux = f"ecart {math.hypot(ex, ey) * 100:.0f} cm"
            elif math.hypot(vx, vy) > VITESSE_XY_MAX:
                douteux = f"vitesse {math.hypot(vx, vy):.1f} m/s"
            elif saut > SAUT_XY_MAX:
                douteux = f"saut de {saut * 100:.0f} cm"
            if douteux:
                xy_actif = False
                evt = (evt + " | " if evt else "") + f"Controle XY arrete (mesure douteuse : {douteux})"
        if xy_actif and x0 is not None:
            # ...ramenes dans le repere du drone (avant = x, gauche = y)
            yaw = math.radians(d.get("stateEstimate.yaw", 0.0))
            c, s_ = math.cos(yaw), math.sin(yaw)
            exb, eyb = c * ex + s_ * ey, -s_ * ex + c * ey
            vxb, vyb = c * vx + s_ * vy, -s_ * vx + c * vy
            # pitch positif (cflib) = penche en avant ; roll positif = penche a droite
            lim = lambda a: max(-ANGLE_CMD_MAX, min(ANGLE_CMD_MAX, a))
            cmd_pitch += lim(-(KP_XY * exb + KD_XY * vxb))
            cmd_roll += lim(KP_XY * eyb + KD_XY * vyb)
        cmd["roll"], cmd["pitch"] = cmd_roll, cmd_pitch
        cf.commander.send_setpoint(cmd_roll, cmd_pitch, 0.0, int(thrust))
        ecrire(w, t, phase, z_cible, z, thrust, e, P, I, D, evt)
        if evt:
            print(f"  [{t:5.2f} s] {evt}")
            evt = ""

        reste = PERIODE - (time.perf_counter() - tb)
        if reste > 0:
            time.sleep(reste)


def main():
    dossier = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
    os.makedirs(dossier, exist_ok=True)
    fichier = os.path.join(dossier, datetime.now().strftime("vol_%Y%m%d_%H%M%S.csv"))

    cflib.crtp.init_drivers()
    print(f"Connexion a {URI} ...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        # IMPORTANT : attendre que tous les parametres soient telecharges AVANT de lancer
        # les logs. Sinon la radio est saturee et param.set_value() bloque (jusqu'a 60 s).
        print("Telechargement des parametres du drone...", flush=True)
        if hasattr(scf, "wait_for_params"):
            scf.wait_for_params()
        print("  parametres OK", flush=True)
        if cf.log.toc.get_element_by_complete_name("range.zrange") is None:
            print("Pas de Flow deck / Z-ranger : impossible de mesurer la hauteur.")
            return

        # Remise a zero de l'estimateur AVANT de lancer les logs (radio libre)
        print("Remise a zero de l'estimateur (ne touche pas le drone, ~3 s)...", flush=True)
        ok1 = set_param(cf, "kalman.resetEstimation", "1")
        time.sleep(0.1)
        ok2 = set_param(cf, "kalman.resetEstimation", "0")
        if ok1 and ok2:
            print("  reset OK", flush=True)
        time.sleep(2.0)

        blocs = [
            # Peu de donnees pour ne pas saturer la radio (sinon les ordres arrivent en retard).
            # Seul le bloc "alt" (utilise par le PID) est rapide : 50 Hz.
            ajouter_bloc(cf, "alt", 20, [("range.zrange", "uint16_t"),
                                         ("stabilizer.roll", "float"),
                                         ("stabilizer.pitch", "float"),
                                         ("stabilizer.thrust", "float")], cb_altitude),
            ajouter_bloc(cf, "imu", 100, [("gyro.x", "float"), ("gyro.y", "float"),
                                         ("gyro.z", "float"), ("acc.z", "float")], cb_simple),
            ajouter_bloc(cf, "pid", 100, [("pid_attitude.roll_outI", "float"),
                                         ("pid_attitude.pitch_outI", "float"),
                                         ("pid_rate.roll_outI", "float"),
                                         ("pid_rate.pitch_outI", "float")], cb_simple),
            ajouter_bloc(cf, "xy", 20, [("stateEstimate.x", "float"), ("stateEstimate.y", "float"),
                                        ("stateEstimate.vx", "float"), ("stateEstimate.vy", "float"),
                                        ("stateEstimate.yaw", "float")], cb_simple),
            ajouter_bloc(cf, "mot", 100, [("motor.m1", "uint16_t"), ("motor.m2", "uint16_t"),
                                         ("motor.m3", "uint16_t"), ("motor.m4", "uint16_t"),
                                         ("pm.vbat", "float"), ("motion.squal", "uint8_t"),
                                         ("supervisor.info", "uint16_t")], cb_simple),
        ]
        time.sleep(1.0)  # laisser arriver les premieres donnees

        with open(fichier, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(COLONNES)
            try:
                raison = voler(cf, w)
            except KeyboardInterrupt:
                import traceback
                traceback.print_exc()   # montre ou le script en etait
                raison = "Ctrl+C (arret manuel)"
            except Exception as ex:
                raison = f"Erreur Python : {ex!r}"
            finally:
                cf.commander.send_stop_setpoint()
                cf.commander.send_notify_setpoint_stop()
            w.writerow([""] * (len(COLONNES) - 1) + ["FIN: " + raison])

        for b in blocs:
            if b:
                b.stop()

    print(f"\n>>> {raison}")
    print(f">>> Log : {fichier}")
    print(">>> Pour analyser : python3 analyse_vol.py")


if __name__ == "__main__":
    main()
