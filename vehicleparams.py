from math import sqrt
import yaml, os

path_cfg = "control.yaml"
f_cfg = open(path_cfg)
car_cfg = yaml.load(f_cfg)

class VehicleParams:
    Lw = car_cfg['Lw']
    Lf = car_cfg['Lf']
    Lr = car_cfg['Lr']
    Lb = car_cfg['v_w'] # width
    Ll = Lw + Lf + Lr # length

    f2x = 1/4 * (3*Lw + 3*Lf - Lr)
    r2x = 1/4 * (Lw + Lf - 3*Lr)
    radius = 1/2 * sqrt((Lw + Lf + Lr) ** 2 / 4 + Lb ** 2)



