import matplotlib.pyplot as plt
from math import *

D = 83              # diamètre de la roue en mm
P = pi * D/1000     # périmètre de la roue en m
N = 3               # nombre d'aimants
T = 0.05            # période d'échantillonnage 
DV = P / (N*T)      # incertitude absolue de vitesse en m/s
INTER_AIMANTS = P/N # distance (périmètre) entre les aimants
DP = 0.02           # dp de position des aimants en m
DTAU = 4e-6         # période de l'horloge interne de l'arduino en s

vms = [i/1000 for i in range(1, 700)]
vkmh = [v*3.6 for v in vms]
dv = [v*sqrt((DP/INTER_AIMANTS)**2+(v*DTAU/INTER_AIMANTS)**2) for v in vms]         # calcul de l'incertitude relative
u = [(DV/v)*100 for v in vms]
lim = [0.1*v for v in vms]
# lim = [10 for v in vms]
percent = round((dv[0]/vms[0])*100, 2)

for i in range(len(vms)):
    # print(f"dv = {format(round(dv[i], 4), ".2f")} m/s | v = {format(round(vms[i], 2), ".2f")} m/s = {format(round(vkmh[i], 2), ".2f")} km/h | dv/v = {percent} %")
    print(f"dv = {format(round(DV, 4), ".2f")} m/s | v = {format(round(vms[i], 2), ".2f")} m/s = {format(round(vkmh[i], 2), ".2f")} km/h | dv/v = {format(round(u[i], 2), ".2f")} %")

plt.figure()
# plt.title(f"Incertitudes de mesure de vitesse\n{format(round(u[len(u)-1], 2), ".2f")} % à {format(round(vms[len(vms)-1], 2), ".2f")} m/s")
plt.title(f"Incertitudes de mesure de vitesse")
plt.xlabel("v (m/s)")
plt.ylabel("Δv (m/s)")
# plt.ylabel("incertitude (%)")
plt.plot(vms, dv, label=f"incertitude relative ({percent} % de la vitesse)")
plt.plot(vms, lim, label=f"incertitude limite (10 % de la vitesse)")
# plt.plot(vms, u, label="incertitude selon la vitesse")
# plt.plot(vms, lim, label="incertitude limite")
# plt.ylim((0, 0.1))
# plt.ylim(0, 25000)
plt.legend()
plt.grid()
plt.show()

# plt.figure()
# plt.xlabel("vitesse (km/h)")
# plt.ylabel("incertitude")
# plt.plot(vkmh, u)
# plt.plot(vkmh, lim)
# plt.ylim((0, 1))
# plt.grid()
# plt.show()
