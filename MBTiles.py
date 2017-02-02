from __future__ import print_function
import sqlite3, zlib

#Implements https://github.com/mapbox/mbtiles-spec/blob/master/1.2/spec.md

class MBTiles(object):
	def __init__(self, filename):

		self.conn = sqlite3.connect(filename)
		self.c = self.conn.cursor()
		self.schemaReady = False

	def __del__(self):
		self.conn.commit()
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

	def CheckSchema(self):		
		sql = "CREATE TABLE IF NOT EXISTS metadata (name text, value text)"
		self.c.execute(sql)

		sql = "CREATE TABLE IF NOT EXISTS tiles (zoom_level integer, tile_column integer, tile_row integer, tile_data blob)"
		self.c.execute(sql)

		self.schemaReady = True

	def GetAllMetaData(self):
		rows = self.c.execute("SELECT name, value FROM metadata")
		out = {}
		for row in rows:
			out[row[0]] = row[1]
		return out

	def SetMetaData(self, name, value):
		if not self.schemaReady:
			self.CheckSchema()

		self.c.execute("UPDATE metadata SET value=? WHERE name=?", (value, name))
		if self.c.rowcount == 0:
			self.c.execute("INSERT INTO metadata (name, value) VALUES (?, ?);", (name, value))

		self.conn.commit()

	def DeleteMetaData(self, name):
		if not self.schemaReady:
			self.CheckSchema()

		self.c.execute("DELETE FROM metadata WHERE name = ?", (name,))
		self.conn.commit()
		if self.c.rowcount == 0:
			raise RuntimeError("Metadata name not found")

	def SetTile(self, zoomLevel, tileColumn, tileRow, data):
		if not self.schemaReady:
			self.CheckSchema()

		self.c.execute("UPDATE tiles SET tile_data=? WHERE zoom_level = ? AND tile_column = ? AND tile_row = ?", 
			(data, zoomLevel, tileColumn, tileRow))
		if self.c.rowcount == 0:
			self.c.execute("INSERT INTO tiles (zoom_level, tile_column, tile_row, tile_data) VALUES (?, ?, ?, ?);", 
				(zoomLevel, tileColumn, tileRow, data))

	def DeleteTile(self, zoomLevel, tileColumn, tileRow):
		if not self.schemaReady:
			self.CheckSchema()

		self.c.execute("DELETE FROM tiles WHERE zoom_level = ? AND tile_column = ? AND tile_row = ?", 
			(data, zoomLevel, tileColumn, tileRow))
		self.conn.commit()

		if self.c.rowcount == 0:
			raise RuntimeError("Tile not found")

	def Commit(self):
		self.conn.commit()

if __name__ == "__main__":
	mbTiles	= MBTiles("andorra.mbtiles")
	metadata = mbTiles.GetAllMetaData()
	for k in metadata:
		print (k, metadata[k])
	#print (mbTiles.ListTiles())
	
	data = mbTiles.GetTile(14, 8275, 10323)
	print ("compressed", len(data))
	print ("uncompressed", len(zlib.decompress(data, 16+zlib.MAX_WBITS)))
	#mbTiles.SetTile(14, 8275, 10323, data)

	metadata = mbTiles.GetAllMetaData()
	mbTiles.SetMetaData("foo", "bar")
	mbTiles.DeleteMetaData("foo")

