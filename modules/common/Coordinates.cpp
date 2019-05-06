#include "modules/common/Coordinates.h"
#include <GeographicLib/GeoCoords.hpp>

using namespace GeographicLib;

/** global to local coordinate conversion
 * @brief Convert a global position (lat, lon, up) to a local poistion
 * (north, east, down) relative to the home positon
 */
V3F global_to_local(const V3F& global_position, const V3F& global_home) {
  GeoCoords pos((double)global_position.x, (double)global_position.y);

  GeoCoords home((double)global_home.x, (double)global_home.y);

  V3F local_position;
  if (pos.Zone() != home.Zone())
    throw "Global and home position are not in same UTM zone.";
  else if (pos.Hemisphere() != home.Hemisphere())
    throw "Global and home positions are not in same Hemisphere.";
  else {
    local_position.x = (float)(pos.Northing() - home.Northing());
    local_position.y = (float)(pos.Easting() - home.Easting());
    local_position.z = -(global_position.z - global_home.z);
  }

  return local_position;
}

/** local to global coordinate conversion
 * @brief Convert a local position (north, east, down) relative to the home
 * position to a global position (lat, lon, up)
 */
V3F local_to_global(const V3F& local_position, const V3F& global_home) {
  GeoCoords home((double)global_home.x, (double)global_home.y);

  double global_north = home.Northing() + (double)local_position.x;

  double global_east = home.Easting() + (double)local_position.y;

  GeoCoords global_position(home.Zone(), home.Northp(), global_east,
                            global_north);

  return V3F((float)global_position.Latitude(),
             (float)global_position.Longitude(),
             global_home.z - local_position.z);
}