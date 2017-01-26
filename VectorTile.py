from __future__ import print_function
from vector_tile21 import Tile
import sys
if sys.version_info < (3,): text = unicode
else: text = str
import math

def ValueToNativePython(value):
	if value.HasField("float_value"):
		return float(value.float_value)
	if value.HasField("double_value"):
		return float(value.double_value)
	if value.HasField("int_value"):
		return int(value.int_value)
	if value.HasField("uint_value"):
		return int(value.uint_value)
	if value.HasField("sint_value"):
		return int(value.sint_value)
	if value.HasField("bool_value"):
		return value.bool_value != 0
	if value.HasField("string_value"):
		return text(value.string_value)
	return None

def NativePythonToValue(value, vp):
	if isinstance(value, int):
		vp.sint_value = int(value)
		return
	if isinstance(value, float):
		vp.float_value = float(value)
		return
	if isinstance(value, bool):
		vp.bool_value = int(value)
		return
	vp.string_value = text(value)
	return

def CheckWindingi(pts):

	total = 0.0
	for i, pt in enumerate(pts):
		i2 = (i+1)%len(pts)
		val = float(pts[i2][0]-pt[0])*float(pts[i2][1] + pt[1])
		total += val
	
	return -total

def CheckCmdId(cmdIdCount, cmdId, count):
	cmdIdCount = cmdIdCount & 0xFFFFFFFF #Limit to 32 bits
	cmdId2 = cmdIdCount & 0x7
	cmdCount2 = cmdIdCount >> 3
	if cmdId != cmdId2 or count != cmdCount2:
		raise OverflowError("Failed to encode command (probably due to number of points)")

# **************************************************************

class DecodeVectorTile(object):
	def __init__(self, tileZoom, tileColumn, tileRow, output):
		self.output = output
		numTiles = pow(2,tileZoom)
		self.latMax, self.lonMin = num2deg(tileColumn, numTiles-tileRow-1, tileZoom)
		self.latMin, self.lonMax = num2deg(tileColumn+1, numTiles-tileRow, tileZoom)
		self.dLat = self.latMax - self.latMin
		self.dLon = self.lonMax - self.lonMin

	def DecodeTileData(self, tileData):
		tile = Tile()

		parsed = tile.ParseFromString(tileData)
		self.output.NumLayers(len(tile.layers))

		for layer in tile.layers:
			self.output.LayerStart(layer.name, layer.version, layer.extent)
			
			#The spec says "Decoders SHOULD parse the version first to ensure that 
			#they are capable of decoding each layer." This has not been implemented.

			for feature in layer.features:

				tagDict = {}
				for tagNum in range(0, len(feature.tags), 2):
					tagDict[layer.keys[feature.tags[tagNum]]] = ValueToNativePython(layer.values[feature.tags[tagNum+1]])

				points, lines, polygons = self.DecodeGeometry(feature, layer.extent)

				fid = None
				if feature.HasField("id"):
					fid = feature.id
			
				self.output.Feature(feature.type, fid, tagDict, 
					points, lines, polygons)

			self.output.LayerEnd()
		
		self.output.Finish()

	def DecodeGeometry(self, feature, extent):
		points = []
		pointsTileSpace = []
		currentPolygon = None
		currentPolygonSet = False
		prevCmdId = 0

		cursorx, cursory = 0, 0
		prevx, prevy = 0.0, 0.0 #Lat lon
		prevxTileSpace, prevyTileSpace = 0, 0
		pointsOut = []
		linesOut = []
		polygonsOut = []
		i = 0

		while i < len(feature.geometry):
			g = feature.geometry[i]
			cmdId = g & 0x7
			cmdCount = g >> 3
			if cmdId == 1: #MoveTo
				for j in range(cmdCount):

					v = feature.geometry[i+1]
					value1 = ((v >> 1) ^ (-(v & 1)))
					v = feature.geometry[i+2]
					value2 = ((v >> 1) ^ (-(v & 1)))
					cursorx += value1
					cursory += value2
					px = self.dLon * float(cursorx) / float(extent) + self.lonMin
					py = - self.dLat * float(cursory) / float(extent) + self.latMax + self.dLat
	
					if feature.type == Tile.POINT:
						pointsOut.append((px, py))

					if feature.type == Tile.LINESTRING and len(points) > 0:
						linesOut.append(points)
						points = []
						pointsTileSpace = []
					
					prevx = px
					prevy = py
					prevxTileSpace = cursorx
					prevyTileSpace = cursory
					i += 2
					prevCmdId = cmdId
				
			if cmdId == 2: #LineTo
			
				for j in range(cmdCount):

					if prevCmdId != 2:
						points.append((prevx, prevy))
						pointsTileSpace.append((prevxTileSpace, prevyTileSpace))
					
					v = feature.geometry[i+1]
					value1 = ((v >> 1) ^ (-(v & 1)))
					v = feature.geometry[i+2]
					value2 = ((v >> 1) ^ (-(v & 1)))
					cursorx += value1
					cursory += value2
					px = self.dLon * float(cursorx) / float(extent) + self.lonMin
					py = - self.dLat * float(cursory) / float(extent) + self.latMax + self.dLat

					points.append((px, py))
					pointsTileSpace.append((cursorx, cursory))
					i += 2
					prevCmdId = cmdId
			
			if cmdId == 7: #ClosePath
			
				#Closed path does not move cursor in v1 to v2.1 of spec.
				# https:#github.com/mapbox/vector-tile-spec/issues/49
				for j in range(cmdCount):

					if feature.type == Tile.POLYGON:
					
						winding = CheckWindingi(pointsTileSpace)
						if winding >= 0.0:
						
							if currentPolygonSet:
								#We are moving on to the next polygon, so store what we have collected so far
								polygonsOut.append(currentPolygon)
								currentPolygon = None
								currentPolygonSet = False
							
							currentPolygon = [points, []] #set outer shape
							currentPolygonSet = True
						
						else:
							currentPolygon[1].append(points) #inner shape
					
						points = []
						pointsTileSpace = []
						prevCmdId = cmdId
						
			i += 1
		
		if feature.type == Tile.LINESTRING:
			linesOut.append(points)
		if feature.type == Tile.POLYGON:
			polygonsOut.append(currentPolygon)
		
		return pointsOut, linesOut, polygonsOut

# *********************************************

# https:#wiki.openstreetmap.org/wiki/Slippy_map_tilenames
def deg2num(lat_deg, lon_deg, zoom):
	lat_rad = math.radians(lat_deg)
	n = 2.0 ** zoom
	xtile = int((lon_deg + 180.0) / 360.0 * n)
	ytile = int((1.0 - math.log(math.tan(lat_rad) + (1 / math.cos(lat_rad))) / math.pi) / 2.0 * n)
	return (xtile, ytile)


def num2deg(xtile, ytile, zoom):
	n = 2.0 ** zoom
	lon_deg = xtile / n * 360.0 - 180.0
	lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * ytile / n)))
	lat_deg = math.degrees(lat_rad)
	return (lat_deg, lon_deg)

# ***********************************************

class DecodeVectorTileResults(object):
	
	def NumLayers(self, numLayers):
		print("Num layers: ", numLayers)

	def LayerStart(self, name, version, extent):
		print ("layer name: ", name)
		print ("layer version: ", version)

	def LayerEnd(self):
		print ("layer end")

	def Feature(self, typeEnum, id, tagDict, points, lines, polygons):

		print (typeEnum, Tile.GeomType.DESCRIPTOR.values_by_number[typeEnum].name, end="")
		if id is not None:
			print (",id=", id, end="")
		print ("")

		for k in tagDict:
			print (text(k).encode("utf-8"), "=", text(tagDict[k]).encode("utf-8"))

		for pt in points:
			print ("POINT(",pt[0],",",pt[1],") ", end="")

		for line in lines:
			print ("LINESTRING(", end="")
			for pt in line:
				print ("(",pt[0],",",pt[1],") ", end="")
			print (") ", end="")
		
		for polygon in polygons:
		
			print ("POLYGON((", end="")
			linePts = polygon[0] #Outer shape
			for pt in linePts:
				print ("(",pt[0],",",pt[1],") ", end="")
			print (")", end="")
			if len(polygon[1]) > 0:
				#Inner shape
				print (",(",end="")
				for lineLoop in polygon[1]:
					print ("(",end="")
					for pt in lineLoop:
						print ("(",pt[0],",",pt[1],") ",end="")
					print (") ",end="")
				print (")",end="")

			print (") ",end="")
		

		print ("")

	def Finish(self):
		pass


# *********************************************************************************

class EncodeVectorTile(DecodeVectorTileResults):
	def __init__(self, tileZoom, tileColumn, tileRow, output): 

		self.tileZoom = tileZoom
		self.tileColumn = tileColumn
		self.tileRow = tileRow
		self.output = output
		self.currentLayer = None
		numTiles = pow(2, tileZoom)
		self.latMax, self.lonMin = num2deg(tileColumn, numTiles-tileRow-1, tileZoom)
		self.latMin, self.lonMax = num2deg(tileColumn+1, numTiles-tileRow, tileZoom)
		self.dLat = self.latMax - self.latMin
		self.dLon = self.lonMax - self.lonMin
		self.tile = Tile()
		self.keysCache = {}
		self.valuesCache = {}
		
	def NumLayers(self, numLayers):
		pass

	def LayerStart(self, name, version, extent):
		if self.currentLayer is not None:
			raise RuntimeError("Previous layer not closed")
		self.currentLayer = self.tile.layers.add()
		self.currentLayer.name = name
		self.currentLayer.version = version
		self.currentLayer.extent = extent

	def LayerEnd(self):

		if self.currentLayer is None:
			raise RuntimeError("Layer not started")
		self.currentLayer = None;
		self.keysCache = {}
		self.valuesCache = {}

	def Feature(self, typeEnum, id, tagDict, points, lines, polygons):
	
		if self.currentLayer is None:
			raise RuntimeError("Cannot add feature: layer not started")
	
		feature = self.currentLayer.features.add()

		if id is not None:
			feature.id = id
		
		for k in tagDict:
			v = tagDict[k]
			vStr = text(v)
		
			try:
				keyIndex = self.keysCache[k]
			except KeyError:
				keyIndex = None
			try:
				valueIndex = self.valuesCache[vStr]
			except KeyError:
				valueIndex = None

			if keyIndex is None:
				kp = self.currentLayer.keys.append(k)
				keyIndex = len(self.currentLayer.keys)-1
				self.keysCache[k] = keyIndex

			if valueIndex is None:
				vp = self.currentLayer.values.add()
				NativePythonToValue(v, vp)
				valueIndex = len(self.currentLayer.values)-1
				self.valuesCache[vStr] = valueIndex

			feature.tags.append(keyIndex)
			feature.tags.append(valueIndex)
		
		#Encode geometry
		self.EncodeGeometry(typeEnum, self.currentLayer.extent, points, lines, polygons, feature)
		
	def Finish(self):
		self.output.write(self.tile.SerializeToString())

	def EncodeTileSpacePoints(self, points, cmdId, reverseOrder, startIndex, cursorPos, outFeature):

		cmdCount = len(points) - startIndex
		cmdIdCount = (cmdId & 0x7) | (cmdCount << 3)
		CheckCmdId(cmdIdCount, cmdId, cmdCount)
		outFeature.geometry.append(cmdIdCount)

		if reverseOrder:
			points = points[::-1]

		for pt in points[startIndex::]:
		
			value1 = pt[0] - cursorPos[0]
			value2 = pt[1] - cursorPos[1]
		
			value1enc = (value1 << 1) ^ (value1 >> 31)
			value2enc = (value2 << 1) ^ (value2 >> 31)

			outFeature.geometry.append(value1enc)
			outFeature.geometry.append(value2enc)

			cursorPos[0] = pt[0]
			cursorPos[1] = pt[1]
		
	def EncodeGeometry(self, typeEnum, extent, points, lines, polygons, outFeature):
	
		if outFeature is None:
			raise RuntimeError("Unexpected null pointer")
		cursorPos = [0, 0]
		outFeature.type = typeEnum

		if typeEnum == Tile.POINT:
		
			tmpTileSpace = self.ConvertToTileCoords(points, extent)
			self.EncodeTileSpacePoints(tmpTileSpace, 1, False, 0, cursorPos, outFeature)
		
		if typeEnum == Tile.LINESTRING:
		
			for line in lines:
			
				tmpTileSpace = self.ConvertToTileCoords(line, extent)
				tmpTileSpace2 = self.DeduplicatePoints(tmpTileSpace)
				if len(tmpTileSpace2) < 2: continue
			
				#Move to start
				tmpPoints = []
				tmpPoints.append(tmpTileSpace2[0])
				self.EncodeTileSpacePoints(tmpPoints, 1, False, 0, cursorPos, outFeature)

				#Draw line shape
				self.EncodeTileSpacePoints(tmpTileSpace2, 2, False, 1, cursorPos, outFeature)
			
		if typeEnum == Tile.POLYGON:
		
			for polygon in polygons:
			
				tmpTileSpace = self.ConvertToTileCoords(polygon[0], extent)
				tmpTileSpace2 = self.DeduplicatePoints(tmpTileSpace)
				if len(tmpTileSpace2) < 2: continue
				reverseOuter = CheckWindingi(tmpTileSpace2) < 0.0;
			
				#Move to start of outer polygon
				if reverseOuter:
					tmpPoints = [tmpTileSpace2[-1]]
				else:
					tmpPoints = [tmpTileSpace2[0]]
				self.EncodeTileSpacePoints(tmpPoints, 1, False, 0, cursorPos, outFeature)

				#Draw line shape of outer polygon
				self.EncodeTileSpacePoints(tmpTileSpace2, 2, reverseOuter, 1, cursorPos, outFeature)

				#Close outer contour
				cmdId = 7
				cmdCount = 1
				cmdIdCount = (cmdId & 0x7) | (cmdCount << 3)
				outFeature.geometry.append(cmdIdCount)

				#Inner polygons
				for inner in polygon[1]:
				
					tmpTileSpace = self.ConvertToTileCoords(inner, extent)
					tmpTileSpace2 = self.DeduplicatePoints(tmpTileSpace)
					if len(tmpTileSpace2) < 2: continue
					reverseInner = CheckWindingi(tmpTileSpace2) >= 0.0

					#Move to start of inner polygon
					if reverseInner:
						tmpPoints = [tmpTileSpace2[-1]]
					else:
						tmpPoints = [tmpTileSpace2[0]]
					self.EncodeTileSpacePoints(tmpPoints, 1, False, 0, cursorPos, outFeature)

					#Draw line shape of inner polygon
					self.EncodeTileSpacePoints(tmpTileSpace2, 2, reverseInner, 1, cursorPos, outFeature)

					#Close inner contour
					cmdId = 7
					cmdCount = 1
					cmdIdCount = (cmdId & 0x7) | (cmdCount << 3)
					outFeature.geometry.append(cmdIdCount)

	def ConvertToTileCoords(self, points, extent):
		out = []
		for pt in points:
			cx = (pt[0] - self.lonMin) * float(extent) / float(self.dLon)
			cy = (pt[1] - self.latMax - self.dLat) * float(extent) / float(-self.dLat)
			out.append((int(round(cx)), int(round(cy))))
		return out
	
	def DeduplicatePoints(self, points):
		out = []
		prevPt = None
		for pt in points:
			if prevPt is None or prevPt != pt:
				out.append(pt);
			prevPt = pt
		return out

