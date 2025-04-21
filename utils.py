import numpy as np
import math



class GPSGeometry():
	def __init__(self):
		self.r_earth = 6378137


	def gps2NEmeter(self, origin_gps, current_gps):
		current_lat, current_lon = current_gps
		origin_lat, origin_lon = origin_gps

		north = (current_lat - origin_lat) / 180 * math.pi * self.r_earth
		east = (current_lon - origin_lon) / 180 * math.pi * math.cos(origin_lat * math.pi / 180) * self.r_earth

		return north, east



	def NEmeter2gps(self, origin_gps, position_coor):
		lat, lon = origin_gps
		north, east = position_coor

		n_lat = lat + (north / self.r_earth) * 180 / math.pi
		n_lon = lon + (east / self.r_earth) * 180 / math.pi / math.cos(lat * math.pi / 180)

		return (n_lat, n_lon)


	def distance(self, gps1, gps2):
		lat1, lon1 = gps1
		lat2, lon2 = gps2

		dLat = lat2 * math.pi / 180 - lat1 * math.pi / 180
		dLon = lon2 * math.pi / 180 - lon1 * math.pi / 180

		a = math.sin(dLat/2) * math.sin(dLat/2) + math.cos(lat1 * math.pi / 180) * math.cos(lat2 * math.pi / 180) * math.sin(dLon/2) * math.sin(dLon/2)
		c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
		d = self.r_earth * c
		return d



# function measure(lat1, lon1, lat2, lon2){  // generally used geo measurement function
#     var R = 6378.137; // Radius of earth in KM
#     var dLat = lat2 * Math.PI / 180 - lat1 * Math.PI / 180;
#     var dLon = lon2 * Math.PI / 180 - lon1 * Math.PI / 180;
#     var a = Math.sin(dLat/2) * Math.sin(dLat/2) +
#     Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
#     Math.sin(dLon/2) * Math.sin(dLon/2);
#     var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
#     var d = R * c;
#     return d * 1000; // meters
# }




if __name__ == '__main__':
	geo = GPSGeometry()
	gps = geo.NEmeter2gps((32.94384937970089, -117.12821707323015), (9.99, 0))
	# answer: 32.94393517529721, -117.12821463907409
	distance = geo.distance(gps, (32.94393517529721, -117.12821463907409))
	print(gps)
	print(distance)
