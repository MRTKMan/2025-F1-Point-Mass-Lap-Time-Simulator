import numpy as np
import matplotlib.animation as animation
import matplotlib.pyplot as plt

# Notes about if chains vs elif chains
# Use if chain if multiple conditions can be true at once and/or you want multiple to be executed at the same time.
# Use elif chain when only one should execute or should be true at the same time.

# Gear Ratio Limits
# Monza 2025:
Gear = 1 # Initial gear
G_1_Limit = 73 * 0.277778 # Limit of Gear 1 converted from kph to m/s
G_2_Limit = 100 * 0.277778 # Limit of Gear 2 converted from kph to m/s
G_3_Limit = 141 * 0.277778 # Limit of Gear 3 converted from kph to m/s
G_4_Limit = 184 * 0.277778 # Limit of Gear 4 converted from kph to m/s
G_5_Limit = 220 * 0.277778 # Limit of Gear 5 converted from kph to m/s
G_6_Limit = 257 * 0.277778 # Limit of Gear 6 converted from kph to m/s
G_7_Limit = 290 * 0.277778 # Limit of Gear 7 converted from kph to m/s
G_8_Limit = float('inf')
t_shift = 0.003

gear_ranges = {
    1: (0, G_1_Limit),
    2: (G_1_Limit, G_2_Limit),
    3: (G_2_Limit, G_3_Limit),
    4: (G_3_Limit, G_4_Limit),
    5: (G_4_Limit, G_5_Limit),
    6: (G_5_Limit, G_6_Limit),
    7: (G_6_Limit, G_7_Limit),
    8: (G_7_Limit, 500) # Used 500 m/s as an arbitrary upper limit for Gear 8, since it is the highest gear and has no defined upper limit.
}

m = 800 # weight limit of F1 car
T = 500 # Max Torque of F1 car in Newton-meters

# Define Track
pi = 3.14159265359
r = 250 # radius of each corner in meters
L = 500 # Length of straights (mini). Long straights are 2*L
xf_a = L  # start finish line to turn 1
xf_b = xf_a + 0.25 * (2*pi*r) # Straight + turn 1
xf_c = xf_b + L # Turn 1 + 500 m mini straight
xf_d = xf_c + 0.25 * (2*pi*r) # 500 m straight + Turn 2
xf_e = xf_d + 2*L # Turn 2 + 1000 m back straight
xf_f = xf_e + 0.25 * (2*pi*r) # Back-straight + Turn 3
xf_g = xf_f + L # Turn 3 + Mini straight
xf_h = xf_g + 0.25 * (2*pi*r) # Mini straight + Turn 4
xf_i = xf_h + 500 # End of lap straight

# Braking Zone Data
d_brake_a = 0 # Initial guess for braking distance
d_brake_b = 0
d_brake_c = 0
d_brake_d = 0

e = 1e-3 # desired error tolerance for v2 and v1 when changing timestep
e_braking = 1e-3 # m/s v2 - v1 error for braking zones
dx_braking = 100 # add 1 meter to braking zone every iteration

# Braking Info
g = 9.81 # acceleration due to gravity in m/s^2
G = 5 # Braking G-force

# Incorporate Rolling Resistance
W = m * g # mass of vehicle
c = 0.02 # coefficient of rollung resistance
F_rr = c * W # Rolling Resistance Force

# Incorporate Drag
rho = 1.225 # air density in kg/m^3
Cd = (0.7922 + 0.8296 + 0.8178 + 0.8066 + 0.7913 + 0.8038) / 6 # average of all 6 Cd values from research paper
Cl = (1.6559 + 1.9034 + 1.7446 + 1.8906 + 1.9925 + 2.1195) / 6 # average of all 6 Cl values from research paper
A_frontal = 1.0 # frontal area in m^2\

# Incorporate Grip Limit
thr = 0 # initialize throttle
br = 1 # initialize brake

# Prove formul

v = 0.1 # initialization of velocity
mew = 1.6 # Peak friction coefficient between tire and road
F_downforce = 0.5 * rho * v**2 * Cl * A_frontal # Weight is positive so downforce sign convention will also be positive
F_tire_max = mew * (W + F_downforce)  # max friction force between tire and road before slip happens
F_c = (m*v**2) / r # centripetal forces
a_longitudinal_max = F_tire_max / m
a_longitudinal_min = - (F_tire_max / m)
v_max_cornering = ( (mew*g) / ((1/r) - ((0.5*mew*rho*A_frontal*Cl)/m) ) ) ** 0.5 # max cornering velocity. Constant
a_cornering_max = (F_tire_max - F_c) / m
a_cornering_min = - (F_tire_max - F_c) / m
# Since max friction force on tire is mew*m*g, a_centripetal formula is v^2/r = mew*m*g / m. Hence the formula above

# Initialize.
dt = 0.1 # timestep size

# Solve for top speed at end of straight and time taken
# Governing Equations
# F = m*a
# P = F*v
# P = m*a*v
# a = P / (m*v)
# Start with a small velocity

# P (Watts) = (RPM * 2pi * Torque (N-m) ) / 60

# Define Track
# start finish straight is 1000 m
# 250 m radius 90 deg corner (Turn 1)
# 500 m straight
# 250 m radius 90 deg corner (Turn 2)
# 1000 m back straight
# 250 m radius corner (Turn 3)
# 500 m straight
# 250 m radius corner (Turn 4)
# Start/finish line starts 500 m into start/finish straight

# Upshifting and Downshifting Logic
def GearShift(v, Gear, t, a, silent=False): # Changes local Gear, not variable Gear.
    # Silent=False prints all be default. You have to set if not silent to not print in a particular case
    # Silent=True doesnt't print by default. You have to set if not silent actually silence it.
    # Silent by default
    # Upshifting Logic
    if v >= G_1_Limit and Gear == 1 and a > 0: # Only upshift if acceleration is positive, to avoid upshifting during braking
        Gear = 2 # Upshift from Gear 1 to gear 2
        t = t + t_shift
        if not silent: # only prints when Silent = False
            print('Upshifted to Gear 2 at time ' + str(t) + ' seconds.')
    if v >= G_2_Limit and Gear == 2 and a > 0: # Only upshift if acceleration is positive, to avoid upshifting during braking  
        Gear = 3 # Upshift from Gear 2 to gear 3
        t = t + t_shift
        if not silent: # only prints when Silent = False
            print('Upshifted to Gear 3 at time ' + str(t) + ' seconds.')
    if v >= G_3_Limit and Gear == 3 and a > 0: # Only upshift if acceleration is positive, to avoid upshifting during braking:
        Gear = 4 # Upshift from Gear 3 to gear 4
        t = t + t_shift
        if not silent: # only prints when Silent = False
            print('Upshifted to Gear 4 at time ' + str(t) + ' seconds.')
    if v >= G_4_Limit and Gear == 4 and a > 0:
        Gear = 5 # Upshift from Gear 4 to gear 5
        t = t + t_shift
        if not silent: # only prints when Silent = False
            print('Upshifted to Gear 5 at time ' + str(t) + ' seconds.')
    if v >= G_5_Limit and Gear == 5 and a > 0:
        Gear = 6 # Upshift from Gear 5 to gear 6
        t = t + t_shift
        if not silent: # only prints when Silent = False
            print('Upshifted to Gear 6 at time ' + str(t) + ' seconds.')
    if v >= G_6_Limit and Gear == 6 and a > 0:
        Gear = 7 # Upshift from Gear 6 to gear 7
        t = t + t_shift
        if not silent: # only prints when Silent = False
            print('Upshifted to Gear 7 at time ' + str(t) + ' seconds.')
    if v >= G_7_Limit and Gear == 7 and a > 0:
        Gear = 8 # Upshift from Gear 7 to gear 8
        t = t + t_shift
        if not silent: # only prints when Silent = False
            print('Upshifted to Gear 8 at time ' + str(t) + ' seconds.')
    # Downshifting Logic
    if v <= G_7_Limit and Gear == 8 and a < 0: # Only downshift if deceleration is positive, to avoid downshifting during acceleration
        Gear = 7 # Downshift from Gear 7 to gear 6
        t = t + t_shift
        if not silent: # only prints when Silent = False
            print('Downshifted to Gear 7 at time ' + str(t) + ' seconds.')
    if v <= G_6_Limit and Gear == 7 and a < 0: # Only downshift if deceleration is positive, to avoid downshifting during acceleration
        Gear = 6 # Downshift from Gear 6 to gear 5
        t = t + t_shift
        if not silent: # only prints when Silent = False
            print('Downshifted to Gear 6 at time ' + str(t) + ' seconds.')
    if v <= G_5_Limit and Gear == 6 and a < 0: # Only downshift if deceleration is positive, to avoid downshifting during acceleration
        Gear = 5 # Downshift from Gear 5 to gear 4
        t = t + t_shift
        if not silent: # only prints when Silent = False
            print('Downshifted to Gear 5 at time ' + str(t) + ' seconds.')
    if v <= G_4_Limit and Gear == 5 and a < 0: # Only downshift if deceleration is positive, to avoid downshifting during acceleration
        Gear = 4 # Downshift from Gear 4 to gear 3
        t = t + t_shift
        if not silent: # only prints when Silent = False
            print('Downshifted to Gear 4 at time ' + str(t) + ' seconds.')
    if v <= G_3_Limit and Gear == 4 and a < 0: # Only downshift if deceleration is positive, to avoid downshifting during acceleration
        Gear = 3 # Downshift from Gear 3 to gear 2
        t = t + t_shift
        if not silent: # only prints when Silent = False
            print('Downshifted to Gear 3 at time ' + str(t) + ' seconds.')
    if v <= G_2_Limit and Gear == 3 and a < 0: # Only downshift if deceleration is positive, to avoid downshifting during acceleration
        Gear = 2 # Downshift from Gear 2 to gear 1
        t = t + t_shift
        if not silent: # only prints when Silent = False
            print('Downshifted to Gear 2 at time ' + str(t) + ' seconds.')
    if v <= G_1_Limit and Gear == 2 and a < 0: # Only downshift if deceleration is positive, to avoid downshifting during acceleration
        Gear = 1 # Downshift from Gear 1 to gear 0
        t = t + t_shift
        if not silent: # only prints when Silent = False
            print('Downshifted to Gear 1 at time ' + str(t) + ' seconds.')
    return Gear, t

# Variable power through RPM range.
def Power(v, Gear): # Returns power output based on velocity and gear
    if Gear == 1 and v <= (29*0.277778): # 29 kph in m/s
        RPM = -7.6346*v**2 - 472.53*v + 8306.4 # R^2 = 0.9991
    elif Gear == 1 and v<= G_1_Limit: # Only runs if the first if statement is false. No risk of redundancy.
        RPM = -20.184*v**2 + 921.82*v - 2028.2 # R^2 = 0.9948
    elif Gear == 2 and v <= G_2_Limit:
        RPM = -9.2081*v**2 + 806.4*v - 5606 # R^2 = 0.9927
    elif Gear == 3 and v <= G_3_Limit:
        RPM = 1.8808*v**3 - 185.03*v**2 + 6107.6*v - 57343 # R^2 = 0.9914
    elif Gear == 4 and v <= G_4_Limit:
        RPM = 4.3426*v**3 - 603.66*v**2 + 27824*v - 413679 # R^2 = 0.902
    elif Gear == 5 and v <= 202*0.277778:
        RPM = -127.13*v**2 + 13519*v - 347615 # R^2 = 0.9044
    elif Gear == 5 and v <= G_5_Limit:
        RPM = -3.0981*v**3 + 554.43*v**2 - 32897*v + 658408 # R^2 = 0.9662
    elif Gear == 6 and v <= 240*0.277778:
        RPM = -57.719*v**2 + 7238*v - 215160 # R^2 = 0.9477
    elif Gear == 6 and v <= G_6_Limit:
        RPM = 0.5602*v**3 -118.94*v**2 + 8539.2*v - 195673 # R^2 = 0.998
    elif Gear == 7 and v <= 263*0.277778:
        RPM = 126.67*v**2 - 18655*v + 697513 # R^2 = 0.9441
    elif Gear == 7 and v <= G_7_Limit:
        RPM = 0.0796*v**3 - 14.008*v**2 + 858.03*v - 8268.5 # R^2 = 0.9937
    elif Gear == 8 and v<=297*0.277778:
        RPM = -621.49*v + 61973 # R^2 = 0.8805
    elif Gear == 8 and v <= G_8_Limit:
        RPM = -0.3263*v**3 + 87.641*v**2 - 7723.3*v + 234551 # R^2 = 0.9978
    else:
        print('RPM range not found')
        RPM = 0    
    return RPM # Provides RPM so it can be used in future lines of code.

# throttle function. Use elif so only one can be true at same time
def throttle(a, x, t, v, a_longitudinal_max, a_cornering_max): # Returns throttle value based on time. 0 = no throttle, 1 = full throttle
    # Be sure to add a_longitudinal max and cornering max to call the local values. Otherwise, it'll call the global values at the top. 
    if t <= 0:     # initialize t:
        thr = 0 # now a local variable
    elif t > 0 and x <= (xf_a - d_brake_a):  # Straight 1 (first half) Grip Limit Before Braking Zone
        thr = 1 # first instint is full throttle
        if a > a_longitudinal_max:
            thr = max(thr - 0.01, 0) # Prevents wheelspin on the straight while preventing throttle from hitting 0
    elif t > 0 and x > (xf_a - d_brake_a) and x <= xf_a: # Straight 1 Braking Zone
        thr = 0 # instantaneous
    elif t > 0 and x > xf_a and x <= xf_b:  # Turn 1 Grip Limit
        thr = 1 # First instint is full throttle
        if v > v_max_cornering or a > a_cornering_max:
            thr = max(thr - 0.01, 0) # Backs off throttle in-corner if going too fast while preventing throttle going below 0.
    elif t > 0 and x > xf_b and x <= (xf_c - d_brake_b):  # Straight 2 Grip Limit Before Braking Zone
        thr = 1 # first instint is full throttle
        if a > a_longitudinal_max:
            thr = max(thr - 0.01, 0) # Prevents wheelspin on the straight while preventing throttle from hitting 0
    elif t > 0 and x > (xf_c - d_brake_b) and x <= xf_c: # Braking Zone Approaching Turn 2
        thr = 0 # Instantaneous
    elif t > 0 and x > xf_c and x <= xf_d:  # Turn 2 Grip Limit
        thr = 1 # First instint if full throttle
        if v > v_max_cornering or a > a_cornering_max:
            thr = max(thr - 0.01, 0) # Backs off throttle in-corner if going too fast while preventing throttle going below 0.
    elif t > 0 and x > xf_d and x <= (xf_e - d_brake_c):  # Straight 3 Grip Limit Before Braking Zone
        thr = 1 # first instint is full throttle
        if a > a_longitudinal_max:
            thr = max(thr - 0.01, 0) # Prevents wheelspin on the straight while preventing throttle from hitting 0
    elif t > 0 and x > (xf_e - d_brake_c) and x <= xf_e: # Straight 3 Braking Zone
        thr = 0 # Instantaneous
    elif t > 0 and x > xf_e and x <= xf_f:  # Turn 3 Grip Limit
        thr = 1 # First instint if full throttle
        if v > v_max_cornering or a > a_cornering_max:
            thr = max(thr - 0.01, 0) # Backs off throttle in-corner if going too fast while preventing throttle going below 0.
    elif t > 0 and x > xf_f and x <= (xf_g - d_brake_d):  # Straight 4 Grip Limit Before Braking Zone
        thr = 1 # first instint is full throttle
        if a > a_longitudinal_max:
            thr = max(thr - 0.01, 0) # Prevents wheelspin on the straight while preventing throttle from hitting 0
    elif t > 0 and x > (xf_g - d_brake_d) and x <= xf_g: # Straight 4 Braking Zone
        thr = 0 # Instantaneous
    elif t > 0 and x > xf_g and x <= xf_h:  # Turn 4 Grip Limit
        thr = 1
        if v > v_max_cornering or a > a_cornering_max:
            thr = max(thr - 0.01, 0) # Backs off throttle in-corner if going too fast while preventing throttle going below 0.
    elif t > 0 and x > xf_h and x <= xf_i:  # Staright 1 (second-half) Grip Limit. End of Lap 1
        thr = 1 # first instint is full throttle
        if a > a_longitudinal_max:
            thr = max(thr - 0.01, 0) # Prevents wheelspin on the straight while preventing throttle from hitting 0
    else:
        print('Error. Throttle has failed because it was called without an appropriate if/else statement.') # Debug
    return thr

# brake pressure function
def braking(a, x, v, thr, a_longitudinal_min): # Return braking value based on acceleratiod. 0 = no brake pressure. 1 = 100% brake pressure.
    if x <= (xf_a - d_brake_a): # Straight 1 Before Braking Zone
        br = 0 # No braking while on a straight
    elif x >= (xf_a - d_brake_a) and x <= xf_a and thr == 0: # Straight 1 Braking Zone
        br = 1 # First reaction is no braking in cornering
        if a < a_longitudinal_min:
            br = br - 0.01 # Reduce brake pressure
    elif x > xf_a and x <= (xf_b): # Turn 1 settings
        br = 0
    elif x > xf_b and x <= (xf_c - d_brake_b): # Straight 2 settings BEFORE braking zone
        br = 0 # First reaction is no braking in cornering
    elif x > (xf_c - d_brake_b) and x <= xf_c and thr == 0: # Straight 2 Braking Zone
        br = 1
        if a < a_longitudinal_min:
            br = br - 0.01 # Reduce brake pressure
    elif x > xf_c and x <= xf_d: # Turn 2 Settings
        br = 0
    elif x > xf_d and x <= (xf_e - d_brake_c): # Straight 3 BEFORE Braking Zone
        br = 0
    elif x > (xf_e - d_brake_c) and x <= xf_e and thr == 0: # Straight 3 Braking Zone
        br = 1 # First reaction is no braking in cornering
        if a < a_longitudinal_min:
            br = br - 0.01 # Reduce brake pressure
    elif x > xf_e and x <= xf_f: # Turn 3 Settings
        br = 0
    elif x > xf_f and x <= (xf_g - d_brake_d): # Straight 4 Before Braking Zone.
        br = 0 # First reaction is no braking in cornering
    elif x > (xf_g - d_brake_d) and x <= xf_g and thr == 0: # Braking Zone Before Turn 4
        br = 1 # First reaction is no braking in cornering
        if a < a_longitudinal_min:
            br = br - 0.01 # Reduce brake pressure    elif x > xf_h and x <= xf_i: # Straight 1 (second half) settings
    elif x > xf_g and x <= xf_h: # Turn 4 Settings
        br = 0
    elif x > xf_h and x <= xf_i: # Straight 1 (Second Half) Settings
        br = 0
    else:
        br = 0
        print('Braking was called at a point where it wasnt supposed to')
    return br

def compute_forces(v): # def(v) bc v is the only variable here. Saves me from having to write it in every while loop segment
    F_drag = 0.5 * rho * v**2 * Cd * A_frontal # Must be before throttle functions bc these determine the griop which then determines the a_max and v_max for throttle function to work
    F_downforce = 0.5 * rho * v**2 * Cl * A_frontal # Weight is positive so downforce sign convention will also be positive
    F_tire_max = mew * (W + F_downforce)  # max friction force between tire and road before slip happens
    F_c = (m*v**2) / r # centripetal forces
    a_longitudinal_max = F_tire_max / m
    a_longitudinal_min = - (F_tire_max / m)
    a_cornering_max = (F_tire_max - F_c) / m
    a_cornering_min = - (F_tire_max - F_c) / m
    return F_drag, F_downforce, F_tire_max, F_c, a_longitudinal_max, a_longitudinal_min, a_cornering_max, a_cornering_min

velocity_data = [] # 
time_data = []
G_force_data = []
a_longitudinal_max_data = []
a_longidudinal_min_data = []
x_linear_data = [] # Obtain linear x_data

def update_data(v, t, a, g, a_longitudinal_max, a_longitudinal_min, x):
    velocity_data.append(v)
    time_data.append(t)
    G_force_data.append(a/g)
    a_longitudinal_max_data.append(a_longitudinal_max/g)
    a_longidudinal_min_data.append(a_longitudinal_min/g) 
    x_linear_data.append(x)

# Convert 1D to 2D
def track_to_cartesian(x):
    # s represents distance travveled in each segment. x is the total distance traveled along the track
    # Pre-compute values that can then be used to calculate the x and y coordinates of the car at any point along the track
    P0 = (0, 0) # Start of track where P0[0] = x-coordinate and P0[1] = y-coordinate
    Pa = (0, L) # End of straight 1
    C1 = (-r, Pa[1]) # Center of Turn 1

    Pb = (-r, Pa[1] + r) # End of Turn 1 # y = L + r
    Pc = (Pb[0] - L, Pb[1]) # End of Straight 2
    C2 = (Pc[0], Pc[1] - r) # Center of Turn 2

    Pd = (Pc[0] - r, Pc[1] - r) # End of Turn 2
    Pe = (Pd[0], Pd[1] - 2*L) # End of Straight 3
    C3 = (Pe[0] + r, Pe[1]) # Center of Turn 3

    Pf = (C3[0] , C3[1] - r) # End of Turn 3
    Pg = (Pf[0] + L, Pf[1]) # End of Straight 4
    C4 = (Pg[0], Pg[1] + r) # Center of Turn 4

    Ph = (C4[0] + r, C4[1]) # End of Turn 4
    Pi = (0, Ph[1] + L) # End of Straight 1 (Second half. Start - finish straight. 
    

    if x <= 0: # Initialize coordinates
        s = 0
        X = P0[0] # local
        Y = P0[1] # local
    elif x > 0 and x <= xf_a: # Straight 1
        s = x # Linear Distance Traveled in this segment
        X = P0[0] # local
        Y = P0[1] + s # local
    elif x > xf_a and x <= xf_b: # Turn 1 Settings
        s = x - xf_a # Linear Distance Traveled in this segment
        theta = 0 + (s / r) # Angle in radians. At xf_b, theta = pi/2
        # Origin for this circle unit circle is -250, 500 and will go from 0 to pi/2
        X = C1[0] + r*np.cos(theta)
        Y = C1[1] + r*np.sin(theta) 
    elif x > xf_b and x <= xf_c: # Straight 2
        s = x - xf_b # Linear Distance Traveled in this segment
        X = Pb[0] - s # local
        Y = Pb[1] # local
    elif x > xf_c and x <= xf_d: # Turn 2 Settings
        s = x - xf_c # Linear Distance Traveled in this segment
        theta = pi/2 + (s / r) # Angle in radians. At xf_d, theta = pi/2
        # Origin for this circle unit circle is -500, 250 and will go from 0 to pi/2
        X = C2[0] + r*np.cos(theta)
        Y = C2[1] + r*np.sin(theta)
    elif x > xf_d and x <= xf_e: # Straight 3
        s = x - xf_d # Linear Distance Traveled in this segment
        X = Pd[0] # local
        Y = Pd[1] - s # local
    elif x > xf_e and x <= xf_f: # Turn 3 Settings
        s = x - xf_e # Linear Distance Traveled in this segment
        theta = pi + (s / r) # Angle in radians. At xf_f, theta = pi/2
        # Origin for this circle unit circle is -250, -500 and will go from 0 to pi/2
        X = C3[0] + r*np.cos(theta)
        Y = C3[1] + r*np.sin(theta)
    elif x > xf_f and x <= xf_g: # Straight 4
        s = x - xf_f # Linear Distance Traveled in this segment
        X = Pf[0] + s # local
        Y = Pf[1] # local
    elif x > xf_g and x <= xf_h: # Turn 4 Settings
        s = x - xf_g # Linear Distance Traveled in this segment
        theta = (3*np.pi)/2 + (s / r) # Angle in radians. At xf_h, theta = pi/2
        # Origin for this circle unit circle is 250, -250 and will go from 0 to pi/2
        X = C4[0] + r*np.cos(theta)   
        Y = C4[1] + r*np.sin(theta)
    elif x > xf_h and x <= xf_i: # Straight 1 (Second half)
        s = x - xf_h # Linear Distance Traveled in this segment
        X = Ph[0]  # local
        Y = Ph[1] + s # local
    return X, Y

def simulate(dt, record=False, decimation=100): # simulate with a starting dt.
    # Decimation=100 means every 100th timestep gets recorded. At a final timestep size of 1E-4, timesteps of 1E-2 for a 50 second lap is sufficient. 
    # default condition is false. If function(record=False) and 
    # if record and.... this block will never run. If it was function(record=True) and
    # if record and... the block would run
    # Initialize Plot Data
    step_count = 0

    # Initialize ALL capture variables. It will say none instead of crashing
    v_a = v_b = v_c = v_d = v_e = v_f = v_g = v_h = v_i = None
    a_a = a_b = a_c = a_d = a_e = a_f = a_g = a_h = a_i = None
    t_a = t_b = t_c = t_d = t_e = t_f = t_g = t_h = t_i = None
    # Braking Zone Data
    global d_brake_a, d_brake_b, d_brake_c, d_brake_d # Since Global variables were required to make brake function work, call those global variables so those will update
    d_brake_a = 50 # Initial guess for braking distance
    d_brake_b = 10
    d_brake_c = 75
    d_brake_d = 10
    dx_braking = 100 # Initialize it as a local variable.

    a_max = - float('inf') # initialize empty min and max. Don't initialize a list bc then it iterates on thousands of entries each time... very slow
    a_min = float('inf')
    v = 0.1 # initial velocity in m/s
    t = 0 # initial time in seconds
    x = 0 # initial position in meters
    Gear = 1 # Start in Gear 1  for better traction
    a = 0 # initial acceleration
    min_power_per_gear = {1: float('inf'), 2: float('inf'), 3: float('inf'), 4: float('inf'), 5: float('inf'), 6: float('inf'), 7: float('inf'), 8: float('inf')} # Initialize min power per gear dictionary. Must to inf so it knows to update to a positive non-zero value.
    max_power_per_gear = {1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0, 7: 0, 8: 0} # Initialize max power per gear dictionary. Must do 0 so it knows to update to a positive non-zero value.

    is_final_pass = False # Have outside whileloop so it can record results on iteration N_final + 1
    while True: # Iterate until told to break.
        # Initialize for every time while Loop iterates
        v = 0.1 # initial velocity in m/s
        t = 0 # initial time in seconds
        x = 0 # initial position in meters
        Gear = 1 # Start in Gear 1  for better traction
        a = 0 # initial acceleration
        while x <= xf_a: # From start finish line to turn 1

            F_drag, F_downforce, F_tire_max, F_c, a_longitudinal_max, a_longitudinal_min, a_cornering_max, a_cornering_min = compute_forces(v)

            # Add braking and centripetal force
            thr = throttle(a, x, t, v, a_longitudinal_max, a_cornering_max) # Get throttle value based on time. Have this before braking since braking is a function of throttle
            br = braking(a, x, v, thr, a_longitudinal_min)
            F_brake = F_tire_max * br # braking force is a function of brake pressure
            # Key Parameters
            Gear, t = GearShift(v, Gear, t, a, silent=True) # Silent=True by default doesn't print, only on your final iteration where the while Loop is true
            RPM = Power(v, Gear) # Get RPM  based on velocity and gear
            if RPM == 0: # Debug
                print('RPM was noted as 0 at time ' + str(t) + ' seconds.')

            P = ((RPM * 2 * 3.14159265359 * T) / 60) * thr # Power in Watts
            # if P == 0: # Debug
            # print('P was noted as 0 at time ' + str(t) + ' seconds.')

            min_power_per_gear[Gear] = min(min_power_per_gear[Gear], P) # Update min power for the current gear if needed
            max_power_per_gear[Gear] = max(max_power_per_gear[Gear], P) # Update max power for the current gear if needed
            a = ((P/v) - F_drag - F_rr - F_brake) / m # Newtons second law --- longitudinal only
            a = min(a, a_longitudinal_max) # cap acceleration in cornering
            a = max(a, a_longitudinal_min) # cap acceleration in braking
            v += a * dt # Update velocity
            if v <= 0: # Debug negative velocity
                v = 0.1 # Prevents negative velocity
                # Don't use break. It will either cost at this minimum speed or accelerate. 
                # Shorter breaking zone makes v < 0 less likely
                # Negative velocity prevents it continuing to next segment.
            x += v * dt # Update position
            t += dt # Update time
            a_max = max(a_max, a) # Not under while column, so should iterate every iteration
            a_min = min(a_min, a)

            # Plotting chart
            if record and is_final_pass and step_count % decimation == 0:
                update_data(v, t, a, g, a_longitudinal_max, a_longitudinal_min, x) # appends data sheet for all of our desired lists if final pass criteria are met
            step_count += 1 # Runs regardless

        if is_final_pass:  # meaning it's true
            break # forces N+1 iterations instead of just N iterations

        elif v > (v_max_cornering): # Approaches corner too fast.  First outcome
            d_brake_a = d_brake_a + dx_braking # Add dx m of braking zone
            # loops again with x <= xf_a loop bc it wasn't told to break.

        elif v < (v_max_cornering - e_braking): # Second outcome. If approaches corner slower than error tolerance.
            dx_braking = dx_braking / 2 # Divide dx braking by 2
            d_brake_a = d_brake_a - dx_braking # reduce braking distance to approach corner faster
            print('car approached corner too slow. Reduce braking zone distance. ')
            if d_brake_a <= 0: 
                d_brake_a = 0
                print('Car did not need to break for turn 1')
                is_final_pass = True # State true before breaking the loop
            if dx_braking <= 0.001:
                is_final_pass = True # State true before braking the loop
            # If it then approaches corner too fast, it will trigger if statement

        elif v >= (v_max_cornering - e_braking) and v <= v_max_cornering: # Enter at a speed between v_corner_max and e_braking
            is_final_pass = True # 
         
    v_a = v # when same intent as while, it activates at the end rather than for every iteration
    a_a = a
    t_a = t
    dx_braking = 100 # Reset dx_braking to global
    print('Car had to brake for Turn 1 for ' + str(d_brake_a) + ' meters.')
    print('Car has entered turn 1 at ' + str(t) + ' seconds and at ' + str(v) + ' m/s')

    while x > xf_a and x <= xf_b: # Shouldn't be any braking in-corner

        F_drag, F_downforce, F_tire_max, F_c, a_longitudinal_max, a_longitudinal_min, a_cornering_max, a_cornering_min = compute_forces(v)

        # Add braking and centripetal force
        thr = throttle(a, x, t, v, a_longitudinal_max, a_cornering_max) # Get throttle value based on time. Must be before braking function
        br = braking(a, x, v, thr, a_longitudinal_min)
        F_brake = F_tire_max * br # braking force is a function of brake pressure

        # Standard Parameters
        Gear, t = GearShift(v, Gear, t, a, silent=False) # Use silent=False bc there's only one iteration here
        RPM = Power(v, Gear) # Get RPM  based on velocity and gear
        if RPM == 0: # Debug
            print('RPM was noted as 0 at time ' + str(t) + ' seconds.')
   
        P = ((RPM * 2 * 3.14159265359 * T) / 60) * thr # Power in Watts    
        min_power_per_gear[Gear] = min(min_power_per_gear[Gear], P) # Update min power for the current gear if needed
        max_power_per_gear[Gear] = max(max_power_per_gear[Gear], P) # Update max power for the current gear if needed
        a = ((P/v) - F_drag - F_rr - F_brake) / m # Newtons second law -- longitudinal only
        a = min(a, a_cornering_max) # cap acceleration in cornering
        a = max(a, a_cornering_min) # cap acceleration in braking
        v += a * dt # Update velocity
        if v <= 0: # Debig negative velocity
            v = 0.1 # Prevents stopping or negative velocity
            # Don't use break. It will either cost at this minimum speed or accelerate. 
            # Shorter breaking zone makes v < 0 less likely
            # Negative velocity prevents it continuing to next segment.
        x += v * dt # Update position
        t += dt # Update time
        a_max = max(a_max, a) # Not under while column, so should iterate every iteration
        a_min = min(a_min, a)

        # For velocity and time plotting. No is_final_pass needed bc corners are one-time only, not iterative
        if record and step_count % decimation == 0:
            update_data(v, t, a, g, a_longitudinal_max, a_longitudinal_min, x) # appends data sheet for all of our desired lists if final pass criteria are met
        step_count += 1 # happens every time regardless

    # This column acts at the end
    dx_braking = 100 # Reset dx_braking
    v_b = v # when same intent as while, it activates at the end rather than for every iteration
    a_b = a
    t_b = t
    Gear_b = Gear
    x_b = x

    print('Car has exited turn 1 at ' + str(t) + ' seconds and at ' + str(v) + ' m/s')

    is_final_pass = False # Have outside while loop so it can record results on iteration N_final + 1
    while True: # Looks forever until told to brake
        # Initialize for every time while Loop iterates
        v = v_b # initial velocity in m/s
        t = t_b # initial time in seconds
        x = x_b # initial position in meters
        a = a_b # initial acceleration
        Gear = Gear_b
        while x > xf_b and x <= xf_c: # Straight 2 Simulation

            F_drag, F_downforce, F_tire_max, F_c, a_longitudinal_max, a_longitudinal_min, a_cornering_max, a_cornering_min = compute_forces(v)

            # Add braking and centripetal force
            thr = throttle(a, x, t, v, a_longitudinal_max, a_cornering_max) # Get throttle value based on time
            br = braking(a, x, v, thr, a_longitudinal_min)
            F_brake = F_tire_max * br # braking force is a function of brake pressure

            # Key Parameters
            Gear, t = GearShift(v, Gear, t, a, silent=True) # Use Silent=True to silence all gearshift prints except the last iteration
            RPM = Power(v, Gear) # Get RPM  based on velocity and gear
            if RPM == 0: # Debug
                print('RPM was noted as 0 at time ' + str(t) + ' seconds.')
   
   
            P = ((RPM * 2 * 3.14159265359 * T) / 60) * thr # Power in Watts
            # if P == 0: # Debug
            # print('P was noted as 0 at time ' + str(t) + ' seconds.')
   
   
            min_power_per_gear[Gear] = min(min_power_per_gear[Gear], P) # Update min power for the current gear if needed
            max_power_per_gear[Gear] = max(max_power_per_gear[Gear], P) # Update max power for the current gear if needed
            a = ((P/v) - F_drag - F_rr - F_brake) / m # Newtons second law
            a = min(a, a_longitudinal_max) # cap acceleration in cornering
            a = max(a, a_longitudinal_min) # cap acceleration in braking
            v += a * dt # Update velocity
            if v <= 0: # Debug negative velocity
                v = 0.1 # Prevents stopping or negative velocity
                # Don't use break. It will either cost at this minimum speed or accelerate. 
                # Shorter breaking zone makes v < 0 less likely
                # Negative velocity prevents it continuing to next segment.

            x += v * dt # Update position
            t += dt # Update time
            a_max = max(a_max, a) # Not under while column, so should iterate every iteration
            a_min = min(a_min, a)

             # Plotting chart Segment
            if record and is_final_pass and step_count % decimation == 0:
                # Since record=False, won't run this block
                update_data(v, t, a, g, a_longitudinal_max, a_longitudinal_min, x) # appends data sheet for all of our desired lists if final pass criteria are met
            step_count += 1 # Runs regardless
        if is_final_pass: # if final_pass is true
            break # ensures N+1 iterations where data records on N+1 

        elif v > (v_max_cornering): # Approaches corner too fast.  First outcome
            d_brake_b = d_brake_b + dx_braking # Add dx m of braking zone
            # loops again with x <= xf_a loop bc it wasn't told to break.
       
        elif v < (v_max_cornering - e_braking): # Second outcome. If approaches corner slower than error tolerance.
            dx_braking = dx_braking / 2 # Divide dx braking by 2
            d_brake_b = d_brake_b - dx_braking # reduce braking distance to approach corner faster
            # If it then approaches corner too fast, it will trigger if statement
            if d_brake_b <= 0: 
                d_brake_b = 0
                print('Car did not need to break for turn 2')
                is_final_pass = True # Alerts when final iteration

            if dx_braking <= 0.001:
                is_final_pass = True # Alerts when final iteration
                   
        elif v >= (v_max_cornering - e_braking) and v <= v_max_cornering: # Enter at a speed between v_corner_max and e_braking
            is_final_pass = True # Alerts when final iteration

    v_c = v # when same intent as while, it activates at the end rather than for every iteration
    a_c = a
    t_c = t
    dx_braking = 100 # Reset dx_braking
    print('Car had to brake for Turn 2 for ' + str(d_brake_b) + ' meters.')
    print('Car has exited straight 2 at ' + str(t) + ' seconds and at ' + str(v) + ' m/s')

    while x > xf_c and x <= xf_d: # Turn 2 Settings. No braking mid-corner

        F_drag, F_downforce, F_tire_max, F_c, a_longitudinal_max, a_longitudinal_min, a_cornering_max, a_cornering_min = compute_forces(v)

        # Add braking and centripetal force
        thr = throttle(a, x, t, v, a_longitudinal_max, a_cornering_max) # Get throttle value based on time
        br = braking(a, x, v, thr, a_longitudinal_min)
        F_brake = F_tire_max * br # braking force is a function of brake pressure
        # Standard Parameters
        Gear, t = GearShift(v, Gear, t, a, silent=False) # Print Silent=False bc there's only one iteration to print
        RPM = Power(v, Gear) # Get RPM  based on velocity and gear
        if RPM == 0: # Debug
            print('RPM was noted as 0 at time ' + str(t) + ' seconds.')
   
        P = ((RPM * 2 * 3.14159265359 * T) / 60) * thr # Power in Watts    
        min_power_per_gear[Gear] = min(min_power_per_gear[Gear], P) # Update min power for the current gear if needed
        max_power_per_gear[Gear] = max(max_power_per_gear[Gear], P) # Update max power for the current gear if needed
        a = ((P/v) - F_drag - F_rr - F_brake) / m # Newtons second law -- longitudinal only
        a = min(a, a_cornering_max) # cap acceleration in cornering
        a = max(a, a_cornering_min) # cap acceleration in braking
        v += a * dt # Update velocity
        if v <= 0: # Debig negative velocity
            v = 0.1 # Keep moving, don't stop
            # Don't use break. It will either cost at this minimum speed or accelerate. 
            # Shorter breaking zone makes v < 0 less likely
            # Negative velocity prevents it continuing to next segment.

        x += v * dt # Update position
        t += dt # Update time
        a_max = max(a_max, a) # Not under while column, so should iterate every iteration
        a_min = min(a_min, a)

        # Data for plotting
        # For velocity and time plotting. No is_final_pass needed bc corners are one-time only, not iterative
        if record and step_count % decimation == 0:
            update_data(v, t, a, g, a_longitudinal_max, a_longitudinal_min, x) # appends data sheet for all of our desired lists if final pass criteria are met
        step_count += 1 # happens every time regardless

    v_d = v # when same intent as while, it activates at the end rather than for every iteration
    a_d = a
    t_d = t
    Gear_d = Gear
    x_d = x
    dx_braking = 100 # Reset dx_braking
    print('Car has exited turn 2 at ' + str(t) + ' seconds and at ' + str(v) + ' m/s')

    is_final_pass = False # Have outside while loop so it can record results on iteration N_final + 1
    while True: # Continue looping until break
        # Initialize for every time while Loop iterates
        v = v_d # initial velocity in m/s
        t = t_d # initial time in seconds
        x = x_d # initial position in meters
        a = a_d # initial acceleration
        Gear = Gear_d
        while x > xf_d and x <= xf_e: # Straight 3 Simulation

            F_drag, F_downforce, F_tire_max, F_c, a_longitudinal_max, a_longitudinal_min, a_cornering_max, a_cornering_min = compute_forces(v)

            # Add braking and centripetal force
            thr = throttle(a, x, t, v, a_longitudinal_max, a_cornering_max) # Get throttle value based on time
            br = braking(a, x, v, thr, a_longitudinal_min)
            F_brake = F_tire_max * br # braking force is a function of brake pressure

            # Key Parameters
            Gear, t = GearShift(v, Gear, t, a, silent=True)
            RPM = Power(v, Gear) # Get RPM  based on velocity and gear
            if RPM == 0: # Debug
                print('RPM was noted as 0 at time ' + str(t) + ' seconds.')
            P = ((RPM * 2 * 3.14159265359 * T) / 60) * thr # Power in Watts
            # if P == 0: # Debug
            # print('P was noted as 0 at time ' + str(t) + ' seconds.')

            min_power_per_gear[Gear] = min(min_power_per_gear[Gear], P) # Update min power for the current gear if needed
            max_power_per_gear[Gear] = max(max_power_per_gear[Gear], P) # Update max power for the current gear if needed
            a = ((P/v) - F_drag - F_rr - F_brake) / m # Newtons second law
            a = min(a, a_longitudinal_max) # cap acceleration in cornering
            a = max(a, a_longitudinal_min) # cap acceleration in braking
            v += a * dt # Update velocity
            if v <= 0: # Debug negative velocity
                v = 0.1 # Prevents stopping or negative velocity
                # Don't use break. It will either cost at this minimum speed or accelerate. 
                # Shorter breaking zone makes v < 0 less likely
                # Negative velocity prevents it continuing to next segment.

            x += v * dt # Update position
            t += dt # Update time
            a_max = max(a_max, a) # Not under while column, so should iterate every iteration
            a_min = min(a_min, a)

             # Plotting chart Segment
            if record and is_final_pass and step_count % decimation == 0:
                # Since record=False, won't run this block
                update_data(v, t, a, g, a_longitudinal_max, a_longitudinal_min, x) # appends data sheet for all of our desired lists if final pass criteria are met
            step_count += 1 # Runs regardless

        if is_final_pass: 
            break # Forces N+1 iterations instead of N iterations 

        elif v > (v_max_cornering): # Approaches corner too fast.  First outcome
            d_brake_c = d_brake_c + dx_braking # Add dx m of braking zone
            # loops again with x <= xf_a loop bc it wasn't told to break.
               
        elif v < (v_max_cornering - e_braking): # Second outcome. If approaches corner slower than error tolerance.
            dx_braking = dx_braking / 2 # Divide dx braking by 2
            d_brake_c = d_brake_c - dx_braking # reduce braking distance to approach corner faster
            # If it then approaches corner too fast, it will trigger if statement
            if d_brake_c <= 0: 
                d_brake_c = 0
                print('Car did not need to break for turn 3')
                is_final_pass = True # Alerts when final iteration

            if dx_braking <= 0.001:
                is_final_pass = True # Alerts when final iteration
                           
        elif v >= (v_max_cornering - e_braking) and v <= v_max_cornering: # Enter at a speed between v_corner_max and e_braking
            is_final_pass = True # Alerts when final iteration

    v_e = v # when same intent as while, it activates at the end rather than for every iteration
    a_e = a
    t_e = t
    dx_braking = 100 # Reset dx_braking
    print('Car had to brake for Turn 3 for ' + str(d_brake_c) + ' meters.')
    print('Car has exited straight 3 at ' + str(t) + ' seconds and at ' + str(v) + ' m/s')

    while x > xf_e and x <= xf_f: # Turn 3 Settings. No braking mid-corner

        F_drag, F_downforce, F_tire_max, F_c, a_longitudinal_max, a_longitudinal_min, a_cornering_max, a_cornering_min = compute_forces(v)

        # Add braking and centripetal force
        thr = throttle(a, x, t, v, a_longitudinal_max, a_cornering_max) # Get throttle value based on time
        br = braking(a, x, v, thr, a_longitudinal_min)
        F_brake = F_tire_max * br # braking force is a function of brake pressure

        # Standard Parameters
        Gear, t = GearShift(v, Gear, t, a, silent=False) # False is ok bc we only need to print one iteration
        RPM = Power(v, Gear) # Get RPM  based on velocity and gear
        if RPM == 0: # Debug
            print('RPM was noted as 0 at time ' + str(t) + ' seconds.')
   
        P = ((RPM * 2 * 3.14159265359 * T) / 60) * thr # Power in Watts    
        min_power_per_gear[Gear] = min(min_power_per_gear[Gear], P) # Update min power for the current gear if needed
        max_power_per_gear[Gear] = max(max_power_per_gear[Gear], P) # Update max power for the current gear if needed
        a = ((P/v) - F_drag - F_rr - F_brake) / m # Newtons second law -- longitudinal only
        a = min(a, a_cornering_max) # cap acceleration in cornering
        a = max(a, a_cornering_min) # cap acceleration in braking
        v += a * dt # Update velocity
        if v <= 0: # Debig negative velocity
            v = 0.1 # Prevents stopping or negative velocity
            # Don't use break. It will either cost at this minimum speed or accelerate. 
            # Shorter breaking zone makes v < 0 less likely
            # Negative velocity prevents it continuing to next segment.

        x += v * dt # Update position
        t += dt # Update time
        a_max = max(a_max, a) # Not under while column, so should iterate every iteration
        a_min = min(a_min, a)
        # For velocity and time plotting. No is_final_pass needed bc corners are one-time only, not iterative
        if record and step_count % decimation == 0:
            update_data(v, t, a, g, a_longitudinal_max, a_longitudinal_min, x) # appends data sheet for all of our desired lists if final pass criteria are met
        step_count += 1 # happens every time regardless
    v_f = v # when same intent as while, it activates at the end rather than for every iteration
    a_f = a
    t_f = t
    x_f = x
    Gear_f = Gear
    dx_braking = 100 # Reset dx_braking
    print('Car has exited turn 3 at ' + str(t) + ' seconds and at ' + str(v) + ' m/s')

    is_final_pass = False # Have outside whileloop so it can record results on iteration N_final + 1
    while True: # Continue looping until break
        # Initialize for every time while Loop iterates
        v = v_f # initial velocity in m/s
        t = t_f # initial time in seconds
        x = x_f # initial position in meters
        a = a_f # initial acceleration
        Gear = Gear_f
        while x > xf_f and x <= xf_g: # Straight 4 Simulation

            F_drag, F_downforce, F_tire_max, F_c, a_longitudinal_max, a_longitudinal_min, a_cornering_max, a_cornering_min = compute_forces(v)

            # Add braking and centripetal force
            thr = throttle(a, x, t, v, a_longitudinal_max, a_cornering_max) # Get throttle value based on time
            br = braking(a, x, v, thr, a_longitudinal_min)
            F_brake = F_tire_max * br # braking force is a function of brake pressure

            # Key Parameters
            Gear, t = GearShift(v, Gear, t, a, silent=True)
            RPM = Power(v, Gear) # Get RPM  based on velocity and gear
            if RPM == 0: # Debug
                print('RPM was noted as 0 at time ' + str(t) + ' seconds.')
   
   
            P = ((RPM * 2 * 3.14159265359 * T) / 60) * thr # Power in Watts
            # if P == 0: # Debug
            # print('P was noted as 0 at time ' + str(t) + ' seconds.')
   
   
            min_power_per_gear[Gear] = min(min_power_per_gear[Gear], P) # Update min power for the current gear if needed
            max_power_per_gear[Gear] = max(max_power_per_gear[Gear], P) # Update max power for the current gear if needed
            a = ((P/v) - F_drag - F_rr - F_brake) / m # Newtons second law
            a = min(a, a_longitudinal_max) # cap acceleration in cornering
            a = max(a, a_longitudinal_min) # cap acceleration in braking
            v += a * dt # Update velocity
            if v <= 0: # Debig negative velocity
                v = 0.1 # Prevents stopping or negative velocity
                # Don't use break. It will either cost at this minimum speed or accelerate. 
                # Shorter breaking zone makes v < 0 less likely
                # Negative velocity prevents it continuing to next segment.

            x += v * dt # Update position
            t += dt # Update time
            a_max = max(a_max, a) # Not under while column, so should iterate every iteration
            a_min = min(a_min, a)

             # Plotting chart Segment
            if record and is_final_pass and step_count % decimation == 0:
                # Since record=False, won't run this block
                update_data(v, t, a, g, a_longitudinal_max, a_longitudinal_min, x) # appends data sheet for all of our desired lists if final pass criteria are met
            step_count += 1 # Runs regardless

        if is_final_pass:
            break # Requires N+1 iterations instead of just N iterations 

        elif v > (v_max_cornering): # Approaches corner too fast.  First outcome
            d_brake_d = d_brake_d + dx_braking # Add dx m of braking zone
            # loops again with x <= xf_a loop bc it wasn't told to break.
               
        elif v < (v_max_cornering - e_braking): # Second outcome. If approaches corner slower than error tolerance.
            dx_braking = dx_braking / 2 # Divide dx braking by 2
            d_brake_d = d_brake_d - dx_braking # reduce braking distance to approach corner faster
            # If it then approaches corner too fast, it will trigger if statement
            if d_brake_d <= 0: 
                d_brake_d = 0
                print('Car did not need to break for turn 4')
                is_final_pass = True # Alerts when final iteration

            if dx_braking <= 0.001:
                is_final_pass = True # Alerts when final iteration

        elif v >= (v_max_cornering - e_braking) and v <= v_max_cornering: # Enter at a speed between v_corner_max and e_braking
            is_final_pass = True # Alerts when final iteration

    v_g = v # when same intent as while, it activates at the end rather than for every iteration
    a_g = a
    t_g = t
    dx_braking = 100 # Reset dx_braking
    print('Car had to brake for Turn 4 for ' + str(d_brake_d) + ' meters.')
    print('Car has exited straight 4 at ' + str(t) + ' seconds and at ' + str(v) + 'm/s')

    while x > xf_g and x <= xf_h: # Turn 4 Settings

        F_drag, F_downforce, F_tire_max, F_c, a_longitudinal_max, a_longitudinal_min, a_cornering_max, a_cornering_min = compute_forces(v)

        # Add braking and centripetal force
        thr = throttle(a, x, t, v, a_longitudinal_max, a_cornering_max) # Get throttle value based on time
        br = braking(a, x, v, thr, a_longitudinal_min)
        F_brake = F_tire_max * br # braking force is a function of brake pressure
        # Standard Parameters
        Gear, t = GearShift(v, Gear, t, a, silent=False) # False is ok bc we only need to print one iteration
        RPM = Power(v, Gear) # Get RPM  based on velocity and gear
        if RPM == 0: # Debug
            print('RPM was noted as 0 at time ' + str(t) + ' seconds.')
   
        P = ((RPM * 2 * 3.14159265359 * T) / 60) * thr # Power in Watts    
        min_power_per_gear[Gear] = min(min_power_per_gear[Gear], P) # Update min power for the current gear if needed
        max_power_per_gear[Gear] = max(max_power_per_gear[Gear], P) # Update max power for the current gear if needed
        a = ((P/v) - F_drag - F_rr - F_brake) / m # Newtons second law -- longitudinal only. Must be before min and max below otherwise it will not be clamped properly and it may use last iteration's a...
        a = min(a, a_cornering_max) # cap acceleration in cornering
        a = max(a, a_cornering_min) # cap acceleration in braking
        v += a * dt # Update velocity
        if v <= 0: # Debig negative velocity
            v = 0.1 # Prevents stopping or negative velocity
            # Don't use break. It will either cost at this minimum speed or accelerate. 
            # Shorter breaking zone makes v < 0 less likely
            # Negative velocity prevents it continuing to next segment.

        x += v * dt # Update position
        t += dt # Update time
        a_max = max(a_max, a) # Not under while column, so should iterate every iteration
        a_min = min(a_min, a)

        # For velocity and time plotting. No is_final_pass needed bc corners are one-time only, not iterative
        if record and step_count % decimation == 0:
            update_data(v, t, a, g, a_longitudinal_max, a_longitudinal_min, x) # appends data sheet for all of our desired lists if final pass criteria are met
        step_count += 1 # happens every time regardless
    dx_braking = 100 # Reset dx_braking
    v_h = v # when same intent as while, it activates at the end rather than for every iteration
    a_h = a
    t_h = t
    print('Car has exited turn 4 at ' + str(t) + ' seconds and at ' + str(v) + 'm/s')

    while x > xf_h and x <= xf_i: # Straight 1 (Second half) settings

        F_drag, F_downforce, F_tire_max, F_c, a_longitudinal_max, a_longitudinal_min, a_cornering_max, a_cornering_min = compute_forces(v)

        # Add braking and centripetal force
        thr = throttle(a, x, t, v, a_longitudinal_max, a_cornering_max) # Get throttle value based on time
        br = braking(a, x, v, thr, a_longitudinal_min)
        F_brake = F_tire_max * br # braking force is a function of brake pressure
        # Key Parameters
        Gear, t = GearShift(v, Gear, t, a, silent=False) # False is ok bc we only need to print one iteration
        RPM = Power(v, Gear) # Get RPM  based on velocity and gear
        if RPM == 0: # Debug
            print('RPM was noted as 0 at time ' + str(t) + ' seconds.')
   
   
        P = ((RPM * 2 * 3.14159265359 * T) / 60) * thr # Power in Watts
        # if P == 0: # Debug
        # print('P was noted as 0 at time ' + str(t) + ' seconds.')
   
   
        min_power_per_gear[Gear] = min(min_power_per_gear[Gear], P) # Update min power for the current gear if needed
        max_power_per_gear[Gear] = max(max_power_per_gear[Gear], P) # Update max power for the current gear if needed
        a = ((P/v) - F_drag - F_rr - F_brake) / m # Newtons second law
        a = min(a, a_longitudinal_max) # cap acceleration in cornering
        a = max(a, a_longitudinal_min) # cap acceleration in braking
        v += a * dt # Update velocity
        if v <= 0: # Debig negative velocity
            v = 0.1 # Prevents stopping or negative velocity
            # Don't use break. It will either cost at this minimum speed or accelerate. 
            # Shorter breaking zone makes v < 0 less likely
            # Negative velocity prevents it continuing to next segment.

        x += v * dt # Update position
        t += dt # Update time
        a_max = max(a_max, a) # Not under while column, so should iterate every iteration
        a_min = min(a_min, a)
        # For velocity and time plotting. No is_final_pass needed bc corners are one-time only, not iterative
        if record and step_count % decimation == 0:
            update_data(v, t, a, g, a_longitudinal_max, a_longitudinal_min, x) # appends data sheet for all of our desired lists if final pass criteria are met
        step_count += 1 # happens every time regardless
    v_i = v
    a_i = a
    t_i = t
    dx_braking = 100 # Reset dx_braking
    print('Car has reached start/finish line at at ' + str(t) + ' seconds and at ' + str(v) + 'm/s')
    # break is unceseasary since not using while True
    # Return must be inside loop

    return t, a, v, Gear, max_power_per_gear, min_power_per_gear, v_a, v_b, v_c, v_d, v_e, v_f, v_g, v_h, v_i, a_a, a_b, a_c, a_d, a_e, a_f, a_g, a_h, a_i, t_a, t_b, t_c, t_d, t_e, t_f, t_g, t_h, t_i, a_max, a_min, d_brake_a, d_brake_b, d_brake_c, d_brake_d, velocity_data, time_data, G_force_data, a_longitudinal_max_data, a_longidudinal_min_data, x_linear_data # Return time, acceleration, velocity, gear, max power per gear, and min power per gear

# Convergence Loop

while True:
    t1, a1, v1, Gear1, max_power_per_gear1, min_power_per_gear1, v_a1, v_b1, v_c1, v_d1, v_e1, v_f1, v_g1, v_h1, v_i1, a_a1, a_b1, a_c1, a_d1, a_e1, a_f1, a_g1, a_h1, a_i1, t_a1, t_b1, t_c1, t_d1, t_e1, t_f1, t_g1, t_h1, t_i1, a_max1, a_min1, d_brake_a1, d_brake_b1, d_brake_c1, d_brake_d1, velocity_data1, time_data1, G_force_data1, a_longitudinal_max_data1, a_longidudinal_min_data1, x_linear_data1  = simulate(dt, record=False) # simulate iteration 1. Must use ALL return variables or it fails.
    dt = dt / 2 # Half timestep size
    t2, a2, v2, Gear2, max_power_per_gear2, min_power_per_gear2, v_a2, v_b2, v_c2, v_d2, v_e2, v_f2, v_g2, v_h2, v_i2, a_a2, a_b2, a_c2, a_d2, a_e2, a_f2, a_g2, a_h2, a_i2, t_a2, t_b2, t_c2, t_d2, t_e2, t_f2, t_g2, t_h2, t_i2, a_max2, a_min2, d_brake_a2, d_brake_b2, d_brake_c2, d_brake_d2, velocity_data2, time_data2, G_force_data2, a_longitudinal_max_data2, a_longidudinal_min_data2, x_linear_data2  = simulate(dt, record=False) # simulate iteration 2. Must use ALL return variables or it fails.
    if abs(t2 - t1) < e and abs(v2 - v1) < e and abs(a2 - a1) < e: # Use t2 and t1 for validation rather than t_i2 and t_i1
        break
    else:
        print('A timestep size of ', dt, ' did not achieve the desired error of 1e-3.')
        continue

print('\nRunning final pass for data recording...') 
(*main_results, # *main_results unpacks all into a lost
 velocity_data, time_data, G_force_data, a_longitudinal_max_data, a_longitudinal_min_data, x_linear_data) = simulate(dt, record=True, decimation=100) # velocity_data and time_data must be in same order as in return ... otherwise it breaks

def plot_track_map(x_data, velocity_data):
    coords = [track_to_cartesian(x) for x in x_linear_data] # converts every x_linear_data into x y pair
    X_cart = [c[0] for c in coords] # list of x coordinates
    Y_cart = [c[1] for c in coords] # list of y coordinates
    fig, ax = plt.subplots(1, 1, figsize=(10, 8)) # fig = whole canvas. ax = the actual plot area. figsize(i,j) = i inches wide, j inches tall.
    fig.suptitle('F1 Oval Track Map', fontsize=14, fontweight='bold')

    ax.plot(X_cart, Y_cart, color='lightgrey', linewidth=10, solid_capstyle='round', zorder=1) # Track outline from all x_cart and y_cart points. round so corners don't look too sharp. 
    sc = ax.scatter(X_cart, Y_cart, c=velocity_data, 
                    cmap = 'RdYlGn', s=5, # V_min goes Rd (red), V_max goes Gn (Green). l (el) not 1 (one)
                    vmin=min(velocity_data), vmax=max(velocity_data))
    # ax.scatter draws dots at each xy position. UNlike ax.plot, which draws a line
    # c=velocity_data means color of each dot is based on velocity_data. vmin and vmax set the min and max of the colorbar scale.
    cbar = plt.colorbar(sc, ax=ax) # color bar. ax=ax to attatch it to your axes.
    cbar.set_label('Velocity (m/s)', fontsize=10)

    # Start/Finish Marker
    ax.plot(0, 0, 'ko', markersize=10, label='Start/Finish')

    ax.set_aspect('equal') # without this, 250 m in X may look dif from 250 m in Y
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.grid(True, alpha=0.2) # alpha 0.2 means 20% transparency. 1 is fully solid. 0 is invisible. 
    ax.legend(fontsize=8)

    plt.tight_layout()
    plt.savefig('track_map.png', dpi=150, bbox_inches='tight') # 72 dpi is low quality, 300 is high quality
    print('Track map saved as track_map.png')
    plt.show() # calls the function which opens the window. MUST have parenthesis

plot_track_map(x_linear_data, velocity_data)

from matplotlib.animation import FuncAnimation # Calls the animation function. Takes plot_track_map repeatively instead of just once. 

def animate_track_map(x_data, velocity_data, time_data): # include time_data otherwise it won't be updated every frame

    # Copy most of the settings 
    coords = [track_to_cartesian(x) for x in x_linear_data] # converts every x_linear_data into x y pair
    X_cart = [c[0] for c in coords] # list of x coordinates
    Y_cart = [c[1] for c in coords] # list of y coordinates
    fig, ax = plt.subplots(1, 1, figsize=(10, 8)) # fig = whole canvas. ax = the actual plot area. figsize(i,j) = i inches wide, j inches tall.
    fig.suptitle('F1 Oval Track Map', fontsize=14, fontweight='bold')
    ax.plot(X_cart, Y_cart, color='lightgrey', linewidth=10, solid_capstyle='round', zorder=1) # Track outline from all x_cart and y_cart points. round so corners don't look too sharp. 
    ax.scatter(X_cart, Y_cart, c=velocity_data, 
                        cmap = 'RdYlGn', s=5, # V_min goes Rd (red), V_max goes Gn (Green). l (el) not 1 (one)
                        vmin=min(velocity_data), vmax=max(velocity_data))
    ax.plot(0, 0, 'ko', markersize=10, label='Start/Finish')         # Start/Finish Marker

    # New info
    dot, = ax.plot(X_cart[0], Y_cart[0], 'ro', markersize=8,
                   zorder=4, label='Car') # red dot for car position. Comma is needed bc it returns a list of lines.
    vel_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, 
                       fontsize=10, verticalalignment='top', 
                       bbox=dict(boxstyle='round', facecolor='white', alpha=0.7)) # Velocity text display in top-left corner
    time_text = ax.text(0.02, 0.90, '', transform=ax.transAxes, # 0.02 is x coordinate and 0.95 is y coordinate
                           fontsize=10, verticalalignment='top', 
                           bbox=dict(boxstyle='round', facecolor='white', alpha=0.7)) # Velocity text display in top-left corner
    # Copy from above
    ax.set_aspect('equal') # without this, 250 m in X may look dif from 250 m in Y
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.grid(True, alpha=0.2) # alpha 0.2 means 20% transparency. 1 is fully solid. 0 is invisible. 
    ax.legend(fontsize=8)

    # Frames per animaton frame. Step=5 = show every 5th frame
    step = 5
    n_frames = len(X_cart) // step # total number of animations

    def update(frame):
        i = frame * step
        dot.set_data([X_cart[i]], [Y_cart[i]]) # Update car position. Must be in [] list form or it fails. 
        vel_text.set_text(f'Velocity: {velocity_data[i] * 3.6 :.2f} kph') # Update velocity text. 2f = 2 decimal places
        time_text.set_text(f'Time: {time_data[i]:.2f} s') # Update time text. 2f = 2 decimal places
        return dot, vel_text, time_text

    anim = FuncAnimation(fig, update, frames=n_frames,
                            interval=50, blit=True)

    plt.tight_layout()
    plt.show()

    return anim

anim = animate_track_map(x_linear_data, velocity_data, time_data)

V_max_cornering = print('For reference, the max cornering speed was ' + str(v_max_cornering) + 'm/s')
Gf = print('The car finished in gear ' + str(Gear2)) # Prints final gear used ---
Gf = Gear2 # Assigns final gear to variable Gf for use in later calculations
tf = print('t_final = ' + str(t_i2) + ' s') # Prints final time in seconds
tf = t_i2
af = print('a_final = ' + str(a_i2) + ' m/s^2') # Prints final acceleration in m/s^2
af = a_i2 # Assigns final acceleration to variable af for use in later calculations
A_max = print('a_max = ' + str(a_max2 / g) + ' Gs ') # Prints final acceleration in m/s^2
A_max = a_max2 # Final acceleration G-force
A_min = print('a_min = ' + str(a_min2 / g) + ' Gs ') # Prints final acceleration in m/s^2
A_min = a_i2 # Assigns final acceleration to variable af for use in later calculations
vf = print('v_final = ' + str(v_i2) + ' m/s') # Prints final velocity in m/s
vf = v_i2 # Assigns final velocity so it can be used in RPM calculator below
vf_mph = v_i2 * 2.23694 # Converts final velocity to miles per hour
vf_mph = print('vf in mph = ' + str(vf_mph) + ' mph')
x_braking1 = print('The braking distance for Turn 1 was ' + str(d_brake_a2) + ' meters.')
x_braking1 = d_brake_a2
x_braking2 = print('The braking distance for Turn 2 was ' + str(d_brake_b2) + ' meters.')
x_braking2 = d_brake_b2
x_braking3 = print('The braking distance for Turn 3 was ' + str(d_brake_c2) + ' meters.')
x_braking3 = d_brake_c2
x_braking4 = print('The braking distance for Turn 4 was ' + str(d_brake_d2) + ' meters.')
x_braking4 = d_brake_d2

# Display min and max RPM Results based on your equations,

for gear in range(1, 9): # Gears 1-8
    v_min, v_max = gear_ranges[gear] # we defined gear max to be 500

    if gear == 8: # overrides m_max with vf in 8th gear only
        v_max = max(v_a2, v_b2, v_c2, v_d2, v_e2, v_f2, v_g2, v_h2, v_i2) # max(a, b) chooses the largest between a and b.
   
    RPM_List = [Power(v, gear) for v in range(int(v_min), int(v_max) + 1)]
    RPM_min = min(RPM_List)
    RPM_max = max(RPM_List)
    print('The min RPM in Gear ' + str(gear) + ' is '+ str(RPM_min) + ' RPM.', end='. ') # Prints min RPM in each gear
    print('The max RPM in Gear ' + str(gear) + ' is '+ str(RPM_max) + ' RPM.') # Prints max RPM in each gear

# Display min and max power output in each gear in kW and HP
for gear in range(1, 9): # Gears 1-8
    # print('The min power output in Gear ' + str(gear) + ' is '+ str(0.001 * min_power_per_gear2[gear]) + ' kW') # Prints min power output in each gear
    # print('The max power output in Gear ' + str(gear) + ' is '+ str(0.001 * max_power_per_gear2[gear]) + ' kW') # Prints max power output in each gear
    print('The min power output in Gear ' + str(gear) + ' is '+ str(0.00134102 * min_power_per_gear2[gear]) + ' HP', end='. ') # Prints min power output in each gear
    print('The max power output in Gear ' + str(gear) + ' is '+ str(0.00134102 * max_power_per_gear2[gear]) + ' HP') # Prints max power output in each gear

# Plotting
def plot_lap(time_data, velocity_data, G_force_data, a_longitudinal_max_data, a_longitudinal_min_data): # Uses axes.___ directly if one plot. If multiple, uses axes[0], axes[1], axes[2]... etc
    # Velocity
    fig, axes = plt.subplots(2, 1, figsize=(14, 5)) # 1,1 means one plot per page. 5,1 does 5 plots per page
    fig.suptitle('F1 Oval Lap Simulator -- Results', fontsize=14, fontweight='bold')

    # Velocity Plots
    # axhline = horizontal line. axvline = vertical line
    axes[0].plot(time_data, velocity_data, 'b-', linewidth=1.5) # Add line for v_max_cornering
    axes[0].axhline(y=v_max_cornering, color='r', linestyle='--',
                    linewidth=1, label=f'v_max_cornering = {v_max_cornering:.1f} m/s') # Add a horizontal line for V_max_cornering Speed
    axes[0].axvline(x=t_a2, color='g', linestyle='--', # Add a label for start of turn 1
                        linewidth=1, label=f'Turn 1 entry = {t_a2:.1f} s') # :.1f turns 96.327 converts to 96.3, .1 = one decimal place
    axes[0].axvline(x=t_b2, color='g', linestyle='--', # Add a label for end of turn 1
                        linewidth=1, label=f'Turn 1 Exit = {t_b2:.1f} s')
    axes[0].axvline(x=t_c2, color='g', linestyle='--', # Add a label for start of turn 2
                        linewidth=1, label=f'Turn 2 Entry = {t_c2:.1f} s')
    axes[0].axvline(x=t_d2, color='g', linestyle='--', # Add a label for end of turn 2
                        linewidth=1, label=f'Turn 2 Exit = {t_d2:.1f} s')
    axes[0].axvline(x=t_e2, color='g', linestyle='--', # Add a label for start of turn 3
                        linewidth=1, label=f'Turn 3 Entry = {t_e2:.1f} s') # 
    axes[0].axvline(x=t_f2, color='g', linestyle='--', # Add a label for end of turn 3
                        linewidth=1, label=f'Turn 3 Exit = {t_f2:.1f} s')
    axes[0].axvline(x=t_g2, color='g', linestyle='--', # Add a label for start of turn 4
                        linewidth=1, label=f'Turn 4 Entry = {t_g2:.1f} s')
    axes[0].axvline(x=t_h2, color='g', linestyle='--', # Add a label for end of turn 4
                        linewidth=1, label=f'Turn 4 Exit = {t_h2:.1f} s')

    axes[0].set_ylabel('Velocity (m/s)')
    axes[0].set_xlabel('Time (s)')
    axes[0].set_title('Velocity vs Time')
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)

  # G-Force Plot
    # axhline = horizontal line. axvline = vertical line
    axes[1].plot(time_data, G_force_data, 'b-', linewidth=1.5) # Add line for v_max_cornering
    axes[1].plot(time_data, a_longitudinal_max_data, 'm-', linestyle='--', 
                 linewidth=1.5, label=f"Max Possible Acceleration G-Force of the tire") # Add line for v_max_cornering
    axes[1].plot(time_data, a_longitudinal_min_data, 'm-', linestyle='--',
                 linewidth=1.5, label=f"Max Possible Braking G-Force of the tire") # Add line for v_max_cornering

    axes[1].axhline(y=0, color='m', linestyle='--',
                    linewidth=1, label=f"Center Line (0 G's)") # Add a horizontal line for V_max_cornering Speed
    axes[1].axvline(x=t_a2, color='g', linestyle='--', # Add a label for start of turn 1
                        linewidth=1) # :.1f turns 96.327 converts to 96.3, .1 = one decimal place
    axes[1].axvline(x=t_b2, color='g', linestyle='--', # Add a label for end of turn 1
                        linewidth=1)
    axes[1].axvline(x=t_c2, color='g', linestyle='--', # Add a label for start of turn 2
                        linewidth=1)
    axes[1].axvline(x=t_d2, color='g', linestyle='--', # Add a label for end of turn 2
                        linewidth=1)
    axes[1].axvline(x=t_e2, color='g', linestyle='--', # Add a label for start of turn 3
                        linewidth=1) # 
    axes[1].axvline(x=t_f2, color='g', linestyle='--', # Add a label for end of turn 3
                        linewidth=1)
    axes[1].axvline(x=t_g2, color='g', linestyle='--', # Add a label for start of turn 4
                        linewidth=1)
    axes[1].axvline(x=t_h2, color='g', linestyle='--', # Add a label for end of turn 4
                        linewidth=1)

    axes[1].set_ylabel("G-Force (G's)")
    axes[1].set_xlabel('Time (s)')
    axes[1].set_title('G-Force vs Time')
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)
    plt.tight_layout() # Gets rid of extra whitespace around edges of figure
    plt.savefig('laptime_simulator_plots.png', dpi=150, bbox_inches='tight') # Saves the figure as a png file. dpi is dots per inch. 
    # bbox_inches='tight' removes extra padding around the chart
    print('Plot saves as laptime_simulation_plots.png')
    plt.show() # Opens a window displaying chart on screen. 

# Colors: 'b' = blue. 'g' = green. 'r' = Red. 'c' = Cyan. 'm' = magenta. 'y' = Yellow. 'k' = Black. 'w' = White

# Call the plot
plot_lap(time_data, velocity_data, G_force_data, a_longitudinal_max_data, a_longitudinal_min_data) # Should plot same order as def plot_lap

