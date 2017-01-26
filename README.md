# pyMbTiles

sudo pip install protobuf

The MBTiles spec is at https://github.com/mapbox/mbtiles-spec

Vector tile spec is at https://github.com/mapbox/vector-tile-spec

UTFGrid spec is at https://github.com/mapbox/utfgrid-spec

MBTiles available at http://osm2vectortiles.org/downloads/ and https://www.mapbox.com/

This software may be redistributed under the MIT license.

# Useful?

To update the protobuf files, remove the line "optimize_for = LITE_RUNTIME;" from vector_tile.proto, then

  protoc vector_tile.proto --python_out vector_tile21


