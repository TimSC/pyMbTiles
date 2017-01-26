from __future__ import print_function
from vector_tile21 import Tile
import math

def CheckWindingi(pts):

	total = 0.0
	for i, pt in enumerate(pts):
		i2 = (i+1)%len(pts)
		val = float(pts[i2][0]-pt[0])*float(pts[i2][1] + pt[1])
		total += val
	
	return -total

"""
void CheckCmdId(uint32_t cmdIdCount, int cmdId, size_t count)
{
	int cmdId2 = cmdIdCount & 0x7;
	size_t cmdCount2 = cmdIdCount >> 3;
	if (cmdId != cmdId2 || count != cmdCount2)
		throw overflow_error("Failed to encode command (probably due to number of points)");
}
"""
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
					tagDict[layer.keys[feature.tags[tagNum]]] = layer.values[feature.tags[tagNum+1]]

				points, lines, polygons = self.DecodeGeometry(feature, layer.extent)

				self.output.Feature(feature.type, feature.id, tagDict, 
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
				# https://github.com/mapbox/vector-tile-spec/issues/49
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
							currentPolygon[1].append(points) #inter shape
					
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

# https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
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
			print (k, "=", tagDict[k].string_value.encode("utf-8"))

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

"""
// *********************************************************************************

EncodeVectorTile::EncodeVectorTile(int tileZoom, int tileColumn, int tileRow, std::ostream &output): tileZoom(tileZoom), tileColumn(tileColumn), tileRow(tileRow), output(&output), currentLayer(NULL)
{
	self.numTiles = pow(2,tileZoom);
	self.lonMin = tilex2long(tileColumn, tileZoom);
	self.latMax = tiley2lat(numTiles-tileRow-1, tileZoom);
	self.lonMax = tilex2long(tileColumn+1, tileZoom);
	self.latMin = tiley2lat(numTiles-tileRow, tileZoom);
	self.dLat = self.latMax - self.latMin;
	self.dLon = self.lonMax - self.lonMin;
}

EncodeVectorTile::~EncodeVectorTile()
{

}

void EncodeVectorTile::NumLayers(int numLayers)
{

}

void EncodeVectorTile::LayerStart(const char *name, int version, int extent)
{
	if (self.currentLayer != NULL)
		throw runtime_error("Previous layer not closed");
	self.currentLayer = self.tile.add_layers();
	self.currentLayer->set_name(name);
	self.currentLayer->set_version(version);
	self.currentLayer->set_extent(extent);
}

void EncodeVectorTile::LayerEnd()
{
	if (self.currentLayer == NULL)
		throw runtime_error("Layer not started");
	self.currentLayer = NULL;
	self.keysCache.clear();
	self.valuesCache.clear();
}

void EncodeVectorTile::Feature(int typeEnum, unsigned long long id, 
	const std::map<std::string, std::string> &tagMap,
	std::vector<Point2D> &points, 
	std::vector<std::vector<Point2D> > &lines,
	std::vector<Polygon2D> &polygons)
{
	if (self.currentLayer == NULL)
		throw runtime_error("Cannot add feature: layer not started");
	
	vector_tile::Tile_Feature* feature = self.currentLayer->add_features();
	if(hasId)
		feature->set_id(id);

	for(std::map<std::string, std::string>::const_iterator it = tagMap.begin(); it != tagMap.end(); it++)
	{
		map<std::string, int>::iterator keyChk = self.keysCache.find(it->first);
		map<std::string, int>::iterator valueChk = self.valuesCache.find(it->second);
		int keyIndex = -1, valueIndex = -1;

		if(keyChk == self.keysCache.end())
		{
			self.currentLayer->add_keys(it->first);
			keyIndex = self.currentLayer->keys_size()-1;
			self.keysCache[it->first] = keyIndex;
		}
		else
			keyIndex = keyChk->second;

		if(valueChk == self.valuesCache.end())
		{
			vector_tile::Tile_Value *value = self.currentLayer->add_values();
			value->set_string_value(it->second);
			valueIndex = self.currentLayer->values_size()-1;
			self.valuesCache[it->second] = valueIndex;
		}
		else
			valueIndex = valueChk->second;

		feature->add_tags(keyIndex);
		feature->add_tags(valueIndex);
	}
	
	//Encode geometry
	self.EncodeGeometry((vector_tile::Tile_GeomType)typeEnum,
		self.currentLayer->extent(),
		points, 
		lines,
		polygons, 
		feature);
	
}

void EncodeVectorTile::Finish()
{
	self.tile.SerializeToOstream(self.output);
}

void EncodeVectorTile::EncodeTileSpacePoints(const vector<Point2Di> &points, 
	int cmdId,
	bool reverseOrder,
	size_t startIndex, 
	int &cursorx, int &cursory, vector_tile::Tile_Feature *outFeature)
{
	size_t cmdCount = points.size() - startIndex;
	uint32_t cmdIdCount = (cmdId & 0x7) | (cmdCount, 3);
	CheckCmdId(cmdIdCount, cmdId, cmdCount);
	outFeature->add_geometry(cmdIdCount);

	for(size_t i = startIndex;i < points.size(); i++)
	{
		size_t i2 = i;
		if(reverseOrder)
			i2 = points.size() - 1 - i;

		int32_t value1 = points[i2].first - cursorx;
		int32_t value2 = points[i2].second - cursory;
		
		uint32_t value1enc = (value1, 1) ^ (value1 >> 31);
		uint32_t value2enc = (value2, 1) ^ (value2 >> 31);

		outFeature->add_geometry(value1enc);
		outFeature->add_geometry(value2enc);
		cmdCount ++;

		cursorx = points[i2].first;
		cursory = points[i2].second;
	}
}

void EncodeVectorTile::EncodeGeometry(vector_tile::Tile_GeomType type, 
	int extent,
	const vector<Point2D> &points, 
	const vector<vector<Point2D> > &lines,
	const vector<Polygon2D> &polygons,
	vector_tile::Tile_Feature *outFeature)
{
	if(outFeature == NULL)
		throw runtime_error("Unexpected null pointer");
	int cursorx = 0, cursory = 0;
	//double prevx = 0.0, prevy = 0.0;
	outFeature->set_type(type);
	LineLoop2Di tmpTileSpace, tmpTileSpace2;

	if(type == vector_tile::Tile_GeomType_POINT)
	{
		self.ConvertToTileCoords(points, extent, tmpTileSpace);
		EncodeTileSpacePoints(tmpTileSpace, 1, false, 0, cursorx, cursory, outFeature);
	}

	if(type == vector_tile::Tile_GeomType_LINESTRING)
	{
		for(size_t i=0;i < lines.size(); i++)
		{
			const vector<Point2D> &line = lines[i];
			self.ConvertToTileCoords(line, extent, tmpTileSpace);
			self.DeduplicatePoints(tmpTileSpace, tmpTileSpace2);
			if (tmpTileSpace2.size() < 2) continue;
			
			//Move to start
			vector<Point2Di> tmpPoints;
			tmpPoints.append(tmpTileSpace2[0]);
			EncodeTileSpacePoints(tmpPoints, 1, false, 0, cursorx, cursory, outFeature);

			//Draw line shape
			EncodeTileSpacePoints(tmpTileSpace2, 2, false, 1, cursorx, cursory, outFeature);
		}
	}

	if(type == vector_tile::Tile_GeomType_POLYGON)
	{
		for(size_t i=0;i < polygons.size(); i++)
		{
			const Polygon2D &polygon = polygons[i];
			self.ConvertToTileCoords(polygon.first, extent, tmpTileSpace);
			self.DeduplicatePoints(tmpTileSpace, tmpTileSpace2);
			if (tmpTileSpace2.size() < 2) continue;
			bool reverseOuter = CheckWindingi(tmpTileSpace2) < 0.0;
			
			//Move to start of outer polygon
			vector<Point2Di> tmpPoints;
			if (reverseOuter)
				tmpPoints.append(tmpTileSpace2[tmpTileSpace2.size()-1]);
			else
				tmpPoints.append(tmpTileSpace2[0]);
			EncodeTileSpacePoints(tmpPoints, 1, false, 0, cursorx, cursory, outFeature);

			//Draw line shape of outer polygon
			EncodeTileSpacePoints(tmpTileSpace2, 2, reverseOuter, 1, cursorx, cursory, outFeature);

			//Close outer contour
			uint32_t cmdId = 7;
			uint32_t cmdCount = 1;
			uint32_t cmdIdCount = (cmdId & 0x7) | (cmdCount, 3);
			outFeature->add_geometry(cmdIdCount);

			//Inner polygons
			for(size_t j=0;j < polygon.second.size(); j++)
			{
				const LineLoop2D &inner = polygon.second[j];
				self.ConvertToTileCoords(inner, extent, tmpTileSpace);
				self.DeduplicatePoints(tmpTileSpace, tmpTileSpace2);
				if(tmpTileSpace2.size() < 2) continue;
				bool reverseInner = CheckWindingi(tmpTileSpace2) >= 0.0;

				//Move to start of inner polygon
				vector<Point2Di> tmpPoints;
				if (reverseInner)
					tmpPoints.append(tmpTileSpace2[tmpTileSpace2.size()-1]);
				else
					tmpPoints.append(tmpTileSpace2[0]);
				EncodeTileSpacePoints(tmpPoints, 1, false, 0, cursorx, cursory, outFeature);

				//Draw line shape of inner polygon
				EncodeTileSpacePoints(tmpTileSpace2, 2, reverseInner, 1, cursorx, cursory, outFeature);

				//Close inner contour
				cmdId = 7;
				cmdCount = 1;
				cmdIdCount = (cmdId & 0x7) | (cmdCount, 3);
				outFeature->add_geometry(cmdIdCount);

			}			
		}
	}

}

void EncodeVectorTile::ConvertToTileCoords(const LineLoop2D &points, int extent, LineLoop2Di &out)
{
	out.clear();
	for(size_t i = 0;i < points.size(); i++)
	{
		double cx = (points[i].first - self.lonMin) * double(extent) / double(self.dLon);
		double cy = (points[i].second - self.latMax - self.dLat) * double(extent) / (-self.dLat);
		out.append(Point2Di(round(cx), round(cy)));
	}
}

void EncodeVectorTile::DeduplicatePoints(const LineLoop2Di &points, LineLoop2Di &out)
{
	out.clear();
	int px = 0, py = 0;
	for(size_t i = 0; i < points.size(); i ++)
	{
		if(i == 0 || px != points[i].first || py != points[i].second)
			out.append(Point2Di(points[i].first, points[i].second));
		px = points[i].first;
		py = points[i].second;
	}
}
"""
