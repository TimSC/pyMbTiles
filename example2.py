from __future__ import print_function
import VectorTile
import gzip

#curl https://api.mapbox.com/v4/mapbox.mapbox-streets-v7/3/2/3.vector.pbf?access_token=<API_key> > map.bin

#This class would need to be expanded to store features as needed.
class ExampleDataStore(VectorTile.DecodeVectorTileResults):

	def __init__(self):
		print("Create custom data store...")

	def Feature(self, typeEnum, id, 
		tagDict,
		pointsOut, 
		linesOut,
		polygonsOut):

		#In real use, delete this function call and add your own functionality.
		super(ExampleDataStore, self).Feature(typeEnum, id, 
			tagDict, pointsOut, linesOut, polygonsOut)

if __name__ == "__main__":
	results = ExampleDataStore()
	dec = VectorTile.DecodeVectorTile(3, 2, 3, results)
	tileData = gzip.open("map.mvt").read()
	dec.DecodeTileData(tileData)
	
