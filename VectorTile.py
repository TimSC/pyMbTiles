from __future__ import print_function
from vector_tile21 import Tile
import math

"""
string FeatureTypeToStr(int typeIn)
{
	::vector_tile::Tile_GeomType type = (::vector_tile::Tile_GeomType)typeIn;
	if(type == ::vector_tile::Tile_GeomType_UNKNOWN)
		return "Unknown";
	if(type == ::vector_tile::Tile_GeomType_POINT)
		return "Point";
	if(type == ::vector_tile::Tile_GeomType_LINESTRING)
		return "LineString";
	if(type == ::vector_tile::Tile_GeomType_POLYGON)
		return "Polygon";
	return "Unknown type";
}

string ValueToStr(const ::vector_tile::Tile_Value &value)
{
	if(value.has_string_value())
		return value.string_value();
	if(value.has_float_value())
	{
		stringstream ss;
		ss, value.float_value();
		return ss.str();
	}
	if(value.has_double_value())
	{
		stringstream ss;
		ss, value.double_value();
		return ss.str();
	}
	if(value.has_int_value())
	{
		stringstream ss;
		ss, value.int_value();
		return ss.str();
	}
	if(value.has_uint_value())
	{
		stringstream ss;
		ss, value.uint_value();
		return ss.str();
	}
	if(value.has_sint_value())
	{
		stringstream ss;
		ss, value.sint_value();
		return ss.str();
	}
	if(value.has_bool_value())
	{
		stringstream ss;
		ss, value.bool_value();
		return ss.str();
	}
	return "Error: Unknown value type";
}

inline double CheckWindingi(LineLoop2Di pts) 
{
	double total = 0.0;
	for(size_t i=0; i < pts.size(); i++)
	{
		size_t i2 = (i+1)%pts.size();
		double val = ((double)(pts[i2].first) - (double)(pts[i].first))*((double)(pts[i2].second) + (double)(pts[i].second));
		total += val;
	}
	return -total;
}

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

"""
		self.output->NumLayers(tile.layers_size());	

		for(int layerNum = 0; layerNum < tile.layers_size(); layerNum++)
		{
			const ::vector_tile::Tile_Layer &layer = tile.layers(layerNum);
			self.output->LayerStart(layer.name().c_str(), layer.version(), layer.extent());

			#The spec says "Decoders SHOULD parse the version first to ensure that 
			#they are capable of decoding each layer." This has not been implemented.

			for(int featureNum = 0; featureNum < layer.features_size(); featureNum++)
			{
				const ::vector_tile::Tile_Feature &feature = layer.features(featureNum);

				map<string, string> tagMap;
				for(int tagNum = 0; tagNum < feature.tags_size(); tagNum+=2)
				{	
					const ::vector_tile::Tile_Value &value = layer.values(feature.tags(tagNum+1));
					tagMap[layer.keys(feature.tags(tagNum))] = ValueToStr(value);
				}

				vector<Point2D> points;
				vector<vector<Point2D> > lines;
				vector<Polygon2D> polygons;
				self.DecodeGeometry(feature, layer.extent(),
					points, lines, polygons);

				self.output->Feature(feature.type(), feature.has_id(), feature.id(), tagMap, 
					points, lines, polygons);
			}

			self.output->LayerEnd();
		}

		self.output->Finish();

void DecodeVectorTile::DecodeGeometry(const ::vector_tile::Tile_Feature &feature,
	int extent,
	vector<Point2D> &pointsOut, 
	vector<vector<Point2D> > &linesOut,
	vector<Polygon2D> &polygonsOut)	
{
	vector<Point2D> points;
	vector<Point2Di> pointsTileSpace;
	Polygon2D currentPolygon;
	bool currentPolygonSet = false;
	unsigned prevCmdId = 0;

	int cursorx = 0, cursory = 0;
	double prevx = 0.0, prevy = 0.0; //Lat lon
	int prevxTileSpace = 0, prevyTileSpace = 0;
	pointsOut.clear();
	linesOut.clear();
	polygonsOut.clear();

	for(int i=0; i < feature.geometry_size(); i ++)
	{
		unsigned g = feature.geometry(i);
		unsigned cmdId = g & 0x7;
		unsigned cmdCount = g >> 3;
		if(cmdId == 1)//MoveTo
		{
			for(unsigned j=0; j < cmdCount; j++)
			{
				unsigned v = feature.geometry(i+1);
				int value1 = ((v >> 1) ^ (-(v & 1)));
				v = feature.geometry(i+2);
				int value2 = ((v >> 1) ^ (-(v & 1)));
				cursorx += value1;
				cursory += value2;
				double px = self.dLon * double(cursorx) / double(extent) + self.lonMin;
				double py = - self.dLat * double(cursory) / double(extent) + self.latMax + self.dLat;
				
				if (feature.type() == vector_tile::Tile_GeomType_POINT)
					pointsOut.push_back(Point2D(px, py));
				if (feature.type() == vector_tile::Tile_GeomType_LINESTRING && points.size() > 0)
				{
					linesOut.push_back(points);
					points.clear();
					pointsTileSpace.clear();
				}
				prevx = px; 
				prevy = py;
				prevxTileSpace = cursorx;
				prevyTileSpace = cursory;
				i += 2;
				prevCmdId = cmdId;
			}
		}
		if(cmdId == 2)//LineTo
		{
			for(unsigned j=0; j < cmdCount; j++)
			{
				if(prevCmdId != 2)
				{
					points.push_back(Point2D(prevx, prevy));
					pointsTileSpace.push_back(Point2D(prevxTileSpace, prevyTileSpace));
				}
				unsigned v = feature.geometry(i+1);
				int value1 = ((v >> 1) ^ (-(v & 1)));
				v = feature.geometry(i+2);
				int value2 = ((v >> 1) ^ (-(v & 1)));
				cursorx += value1;
				cursory += value2;
				double px = self.dLon * double(cursorx) / double(extent) + self.lonMin;
				double py = - self.dLat * double(cursory) / double(extent) + self.latMax + self.dLat;

				points.push_back(Point2D(px, py));
				pointsTileSpace.push_back(Point2D(cursorx, cursory));
				i += 2;
				prevCmdId = cmdId;
			}
		}
		if(cmdId == 7) //ClosePath
		{
			//Closed path does not move cursor in v1 to v2.1 of spec.
			// https://github.com/mapbox/vector-tile-spec/issues/49
			for(unsigned j=0; j < cmdCount; j++)
			{
				if (feature.type() == vector_tile::Tile_GeomType_POLYGON)
				{
					double winding = CheckWindingi(pointsTileSpace);
					if(winding >= 0.0)
					{
						if(currentPolygonSet)
						{
							//We are moving on to the next polygon, so store what we have collected so far
							polygonsOut.push_back(currentPolygon);
							currentPolygon.first.clear();
							currentPolygon.second.clear();
						}
						currentPolygon.first = points; //outer shape
						currentPolygonSet = true;
					}
					else
						currentPolygon.second.push_back(points); //inter shape
					
					points.clear();
					pointsTileSpace.clear();
					prevCmdId = cmdId;
				}				
			}
		}
	}

	if (feature.type() == vector_tile::Tile_GeomType_LINESTRING)
		linesOut.push_back(points);
	if (feature.type() == vector_tile::Tile_GeomType_POLYGON)
		polygonsOut.push_back(currentPolygon);
}
"""
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

	def Feature(self, typeEnum, hasId, id, tagDict, points, lines, polygons):

		print (typeEnum, FeatureTypeToStr(typeEnum), end="")
		if hasId:
			print (",id=", id, end="")
		print ("")

		for k in tagDict:
			print (k, "=", tagDict[k])

		for pt in points:
			print ("POINT(",pt[0],",",pt[1],") ", end="")

		for line in lines:
			print ("LINESTRING(", end="")
			for pt in line:
				print ("(",pt[0],",",pt[1],") ", end="")
			print (") ", end="")
			"""
		for(size_t i =0; i < polygons.size(); i++)
		{
			Polygon2D &polygon = polygons[i];
			print ("POLYGON((";
			LineLoop2D &linePts = polygon.first; //Outer shape
			for(size_t j =0; j < linePts.size(); j++)
				print ("("<<linePts[j].first<<","<<linePts[j].second<<") ";
			print (")";
			if(polygon.second.size() > 0)
			{
				//Inner shape
				print (",(";
				for(size_t k =0; k < polygon.second.size(); k++)
				{
					print ("(";
					LineLoop2D &linePts2 = polygon.second[k];
					for(size_t j =0; j < linePts2.size(); j++)
						print ("("<<linePts2[j].first<<","<<linePts2[j].second<<") ";
					print (") ";
				}
				print (")";
			}

			print (") ";
		}
"""
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

void EncodeVectorTile::Feature(int typeEnum, bool hasId, unsigned long long id, 
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
			tmpPoints.push_back(tmpTileSpace2[0]);
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
				tmpPoints.push_back(tmpTileSpace2[tmpTileSpace2.size()-1]);
			else
				tmpPoints.push_back(tmpTileSpace2[0]);
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
					tmpPoints.push_back(tmpTileSpace2[tmpTileSpace2.size()-1]);
				else
					tmpPoints.push_back(tmpTileSpace2[0]);
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
		out.push_back(Point2Di(round(cx), round(cy)));
	}
}

void EncodeVectorTile::DeduplicatePoints(const LineLoop2Di &points, LineLoop2Di &out)
{
	out.clear();
	int px = 0, py = 0;
	for(size_t i = 0; i < points.size(); i ++)
	{
		if(i == 0 || px != points[i].first || py != points[i].second)
			out.push_back(Point2Di(points[i].first, points[i].second));
		px = points[i].first;
		py = points[i].second;
	}
}
"""
