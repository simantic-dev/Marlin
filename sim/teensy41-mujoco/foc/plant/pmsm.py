# Rotor state per servo axis, shared by foc_plant.py (which integrates it) and mt6835_peer.py (which reads the
# shaft angle): a module of its own so both peer scripts import the same instance.
P = 4  # Nanotec DB42S02: 8 poles


class Motor:
    def __init__(self):
        self.id = self.iq = 0.0
        self.wm = 0.0
        self.load = 0.0  # N.m opposing the rotor, set by the belt in peers/printer_plant.py
        self.theta_m = 0.6 / P  # rotor starts 0.6 electrical rad off the d-axis alignment


MOTORS = {"x": Motor(), "y": Motor()}  # same datasheet motor on each servo axis
