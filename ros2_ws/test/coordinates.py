import math as m

###

lon_degree_to_meter = lambda latitude : 40075000 * m.cos(m.radians(latitude)) / 360  

meter_to_lat_degree = lambda meters: meters / 111320
meter_to_lon_degree = lambda meters, latitude: meters / (lon_degree_to_meter(latitude))

current = [65.6166222, 22.1373866]
plus_meters = 2
sign = 1

###

plus_current = current.copy()
plus_current[0] = current[0] + sign * meter_to_lat_degree(plus_meters)
plus_current[1] = current[1] + sign * meter_to_lon_degree(plus_meters, current[0])

print(f'New Coordinates: {plus_current}')