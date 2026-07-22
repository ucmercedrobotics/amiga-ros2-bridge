from typing import Dict, List, Any, Tuple
import argparse

import utils

def write_kml(msgs: List[Any], out_file: str, plot_every: int = 1):
    with open(out_file, 'w') as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write('<kml xmlns="http://www.opengis.net/kml/2.2">\n')
        f.write('  <Document>\n')
        f.write('    <name>Waypoints</name>\n')
        f.write('    <Placemark>\n')
        f.write('      <LineString>\n')
        f.write('        <coordinates>\n')
        for i in range(0, len(msgs), plot_every):
            msg = msgs[i]
            lon, lat, alt = msg.longitude, msg.latitude, msg.altitude
            f.write(f'          {lon},{lat},{alt}\n')
        f.write('        </coordinates>\n')
        f.write('      </LineString>\n')
        f.write('    </Placemark>\n')
        f.write('  </Document>\n')
        f.write('</kml>\n')

def main():
    parser = argparse.ArgumentParser(
                prog='KML Utility',
                description='Writes NavSatFix messages to a .kml file to use with Google Earth'
            )
    parser.add_argument('bagfile_path')           
    parser.add_argument('-o', '--out_path', type=str, default="waypoints.kml")
    parser.add_argument('-p', '--plot_every', type=int, default=1)
    args = parser.parse_args()
    
    # -- Parse ROSBAG
    topics = [
        '/ublox_gps_node/fix',
    ]
    bag = utils.parse_bag(args.bagfile_path, topics)
    
    # -- Plot Local EKF, Global EKF, and GPS
    write_kml(bag['/ublox_gps_node/fix'], args.out_path, args.plot_every)
    
         
if __name__ == "__main__":
    main()