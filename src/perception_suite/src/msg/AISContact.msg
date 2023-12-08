# AIS Contact Data

Header header              # msg timestamp, frame_id = ais

time     original_time     # original UTC timestamp of AIS message
uint32   pgn               # id of the NMEA 2000 message
uint32   mmsi    	         # always present
uint8    msg_id  	         # message id (indicates kind of class A or B message)
string   name    	         # vessels name (empty = NA)
string   callsign	         # vessels callsign (empty = NA)
uint32   imo	   	         # IMO ship identification number (0 = NA)
string   nav_status  	     # Navigational status [empty = NA, "Under way using engine",
			   # "At anchor", "Not under command", "Restricted maneuverability",
			   # "Constrained by her draught", "Moored", Aground",
			   # "Engaged in Fishing", "Under way sailing", 
			   # "Hazardous material - High Speed", "Hazardous material - Wing in Ground",
			   # "AIS-SART"]
float32  rot	   	         # Rate of Turn (-128 to 127, where -128 = not available)
float32  sog		           # Speed Over Ground in m/s, 53.0 = NA
float32  cog		           # Course over ground in degrees (0-359.9), 360.0 = NA
string 	 pac		           # Position Accuracy ["Low" = default, "Low", "High"]
float32	 true_heading	     # Degrees (0-359), 511 indicates NA = default
uint32	 UTC_second	       # UTC second (0-59), 60=Not available, 61=Manual input mode, 62=Dead 			   
                           # reckoning mode, 63=Positioning system is inoperative
float32  longitude	       # in fractional degrees, 360.0 = default = NA
float32	 latitude	         # in fractional degrees, 360.0 = default = NA
string 	 special_maneuver  # [empty = NA, "Not engaged in special maneuver", 
                           # "Engaged in special maneuver", "Reserved"]
                           # Note: Raw JSON uses "Not available"
string	 gnss              # ["" = NA, "GPS", "GLONASS", "GPS+GLONASS", "Loran-C",
	 		                     # "Chayka", "integrated", "surveyed","Galileo"]
float32	 length            # Length in m, 0 = NA
float32  beam              # Beam in m, 0 = NA
float32  draft             # Draft in m, 0 = NA
float32	 distance_bow      # A: distance from ref position to bow in meters
	 		                     # (0 to 511, 512 = NA)
float32  distance_stbd     # D: distance from ref position to stbd in meters
	 		                     # (0 to 63, 64 = NA)

