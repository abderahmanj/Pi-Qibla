#equation
#TODO: function which get degrees to qibla from current GPS coordinates
#currently returns test data
import math
def get_qibla_degrees(coords):
    Xp = coords[0]
    Yp = coords[1]
    #get qibla degrees for above lat lon
    Xm = 21.3891*(math.pi/180)
    Ym = 39.8579*(math.pi/180)
    a = math.sin(Ym-Yp)
    b = math.cos(Xp)
    c = math.tan(Xm)
    d = math.sin(Xp)
    e = math.cos(Ym-Yp)
    f = b*c
    g = d*e
    qibla_degrees = math.atan2((a),(f-g))
    
    return math.degrees(qibla_degrees)

coords = [40*(math.pi/180), -74*(math.pi/180)]
qibla_degrees = get_qibla_degrees(coords)
print(qibla_degrees)

