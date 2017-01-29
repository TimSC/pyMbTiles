from __future__ import print_function
import VectorTile
import gzip, sys

#curl https://api.mapbox.com/v4/mapbox.mapbox-streets-v7/3/2/3.vector.pbf?access_token=<API_key> > map.bin

#This class would need to be expanded to store features as needed.
class ExampleDataStore(VectorTile.DecodeVectorTileResults):

	def __init__(self):
		print("Create custom data store...")

	def Feature(self, typeEnum, id, tagDict,
		pointsOut, linesOut, polygonsOut):

		#In real use, delete this function call and add your own functionality.
		super(ExampleDataStore, self).Feature(typeEnum, id, 
			tagDict, pointsOut, linesOut, polygonsOut)

if __name__ == "__main__":
	finaIn = "map.mvt"
	finaOut = "mapout.mvt"
	tileZoom = 3
	tileColumn = 2
	tileRow = 3
	if len(sys.argv) >= 2:
		finaIn = sys.argv[1]
	if len(sys.argv) >= 3:
		finaOut = sys.argv[2]
	if len(sys.argv) >= 4:
		tileZoom = int(sys.argv[3])
	if len(sys.argv) >= 5:
		tileColumn = int(sys.argv[4])
	if len(sys.argv) >= 6:
		tileRow = int(sys.argv[5])

	#Print decoded data to screen
	results = ExampleDataStore()
	dec = VectorTile.DecodeVectorTile(tileZoom, tileColumn, tileRow, results)
	tileData = gzip.open(finaIn).read()
	dec.DecodeTileData(tileData)

	#Reencode to file
	encData = gzip.open(finaOut, "wb")
	enc = VectorTile.EncodeVectorTile(tileZoom, tileColumn, tileRow, encData)
	dec = VectorTile.DecodeVectorTile(tileZoom, tileColumn, tileRow, enc)
	tileData = gzip.open(finaIn).read()
	dec.DecodeTileData(tileData)
	
	encData.close()

