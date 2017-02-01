from __future__ import print_function
import sqlite3

class MBTiles(object):
	def __init__(self, filename):

		self.conn = sqlite3.connect(filename)
		self.c = self.conn.cursor()

		rows = self.c.execute("SELECT name, value FROM metadata")
		self.metadata = {}
		for row in rows:
			self.metadata[row[0]] = row[1]

	def __del__(self):
		self.c.close()
		del self.conn

	def ListTiles(self):
		rows = self.c.execute("SELECT zoom_level, tile_column, tile_row FROM tiles")
		out = []
		for row in rows:
			out.append((row[0], row[1], row[2]))
		return out

	def GetTile(self, zoomLevel, tileColumn, tileRow):

		rows = self.c.execute("SELECT tile_data FROM tiles WHERE zoom_level = ? AND tile_column = ? AND tile_row = ?", 
			(zoomLevel, tileColumn, tileRow))
		rows = list(rows)
		if len(rows) == 0:
			raise RuntimeError("Tile not found")
		row = rows[0]
		return row[0]

if __name__ == "__main__":
	mbTiles	= MBTiles("andorra.mbtiles")
	print (mbTiles.metadata)
	print (mbTiles.ListTiles())
	
	data = mbTiles.GetTile(14, 8275, 10323)
	print (len(data))


