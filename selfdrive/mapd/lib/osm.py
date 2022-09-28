import overpy
import numpy as np
from selfdrive.mapd.lib.geo import R
import os


_SIM = "SIMULATION" in os.environ
_DEBUG = True


def _debug(msg):
  if not _DEBUG:
    return
  print(msg)


class OSM():
  def __init__(self):
    self.api = overpy.Overpass()
    # self.api = overpy.Overpass(url='http://3.65.170.21/api/interpreter')

  def fetch_road_ways_around_location(self, lat, lon, radius):
    if _SIM:
      return self.simulator_fetch()

    # Calculate the bounding box coordinates for the bbox containing the circle around location.
    bbox_angle = np.degrees(radius / R)
    # fetch all ways and nodes on this ways in bbox
    bbox_str = f'{str(lat - bbox_angle)},{str(lon - bbox_angle)},{str(lat + bbox_angle)},{str(lon + bbox_angle)}'
    lat_lon = "(%f,%f)" % (lat, lon)
    q = """
        way(""" + bbox_str + """)
          [highway]
          [highway!~"^(footway|path|corridor|bridleway|steps|cycleway|construction|bus_guideway|escape|service|track)$"];
        (._;>;);
        out;""" + """is_in""" + lat_lon + """;area._[admin_level~"[24]"];
        convert area ::id = id(), admin_level = t['admin_level'],
        name = t['name'], "ISO3166-1:alpha2" = t['ISO3166-1:alpha2'];out;
        """
    try:
        query = self.api.query(q)
        areas, ways = query.areas, query.ways
    except Exception as e:
      print(f'Exception while querying OSM:\n{e}')
      areas, ways = [],[]

    return areas, ways

  def simulator_fetch(self):
    _debug('OSM: Start simulator fetch')

    path = os.getenv("CARLA_OVERPASS_XML_PATH", "../mapd/lib/carla_town04_osm_overpass.xml")

    with open(path, 'r') as f:
      overpass_xml = f.read()
      ways = self.api.parse_xml(overpass_xml).ways

    _debug('OSM: Finish simulator fetch')
    return ways
