# https://towardsdatascience.com/host-your-own-offline-mapping-server-with-jupyter-notebook-ff21b878b4d7

import math
from math import pow
import os
import os.path
import requests 
import json

from flask import Flask
from flask import render_template
from flask import request
from werkzeug.wrappers import Request, Response

def num2deg(xtile, ytile, zoom):
    n = 2.0 ** zoom
    lon_deg = xtile / n * 360.0 - 180.0
    lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * ytile / n)))
    lat_deg = math.degrees(lat_rad)
    return (lat_deg, lon_deg)

def deg2num(lat_deg, lon_deg, zoom):
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return (xtile, ytile)

def makeGeoJSON(lat_deg, lon_deg, zoom):
    feature={
    "type":"Feature",
    "geometry": {
      "type": "Point",
      "coordinates": [
        lon_deg,
        lat_deg
      ]
     },
      "properties": {
        "zoom": zoom
      } 
    }
    return feature

def getFromLocalServer(lat_deg, lon_deg, zoom):
    baseMapUrlPrefix = "http://localhost:3650/api/tiles/osm-2020-02-10-v3/"
    baseMapUrlSuffix = ''

    # y,x = deg2num(lat_deg, lon_deg, zoom)
    x = 0
    y = 0
    z = 0

    baseMapTileUrl=baseMapUrlPrefix+str(zoom) + "/" + str(x) + "/" + str(y) + baseMapUrlSuffix
    r = requests.get(baseMapTileUrl)

    # data = makeGeoJSON(lat_deg, lon_deg, zoom)
    # r = requests.post(baseMapUrlPrefix, json=json.dumps(data))

    print(r)
    print(r.content)

for i in range(1):
    getFromLocalServer(-72.127765,41.916669999999996,i)

# # Specify Zoom Level i.e.The minimum zoom level in Step 1:
# zoom=11

# # initialise a GeoJSON Feature Collection object
# geojsonObj={
#   "type": "FeatureCollection",
#   "features": []
# }

# # Copy and Paste the output from Step 1 and store it into the variable xy:
# xy=[{"x":1204,"y":1560},{"x":1204,"y":1559},{"x":1203,"y":1560},{"x":1205,"y":1560},
#     {"x":1204,"y":1561},{"x":1203,"y":1559},{"x":1205,"y":1559},{"x":1203,"y":1561},
#     {"x":1205,"y":1561},{"x":1202,"y":1560},{"x":1206,"y":1560},{"x":1202,"y":1559},
#     {"x":1206,"y":1559},{"x":1202,"y":1561},{"x":1206,"y":1561},{"x":1201,"y":1560},
#     {"x":1207,"y":1560},{"x":1201,"y":1559},{"x":1207,"y":1559},{"x":1201,"y":1561},
#     {"x":1207,"y":1561},{"x":1203,"y":1558},{"x":1202,"y":1558},{"x":1204,"y":1558},
#     {"x":1201,"y":1558},{"x":1205,"y":1558},{"x":1200,"y":1559},{"x":1200,"y":1558},
#     {"x":1206,"y":1558},{"x":1200,"y":1560},{"x":1199,"y":1559},{"x":1199,"y":1558},
#     {"x":1199,"y":1560},{"x":1200,"y":1561},{"x":1199,"y":1561},{"x":1198,"y":1559},
#     {"x":1198,"y":1560},{"x":1198,"y":1558},{"x":1198,"y":1561},{"x":1197,"y":1559},
#     {"x":1197,"y":1558},{"x":1197,"y":1560},{"x":1196,"y":1559},{"x":1196,"y":1558},
#     {"x":1196,"y":1560},{"x":1205,"y":1559},{"x":1206,"y":1559},{"x":1205,"y":1560},
#     {"x":1206,"y":1560},{"x":1204,"y":1559},{"x":1204,"y":1560},{"x":1204,"y":1558},
#     {"x":1205,"y":1558},{"x":1203,"y":1558},{"x":1203,"y":1559},{"x":1202,"y":1559},
#     {"x":1202,"y":1558},{"x":1202,"y":1560},{"x":1203,"y":1560},{"x":1201,"y":1559},
#     {"x":1201,"y":1560},{"x":1201,"y":1558},{"x":1200,"y":1558},{"x":1200,"y":1559},
#     {"x":1205,"y":1560},{"x":1206,"y":1560},{"x":1205,"y":1561},{"x":1206,"y":1561},
#     {"x":1204,"y":1560},{"x":1204,"y":1561},{"x":1204,"y":1559},{"x":1205,"y":1559},
#     {"x":1203,"y":1559},{"x":1203,"y":1560},{"x":1202,"y":1560},{"x":1202,"y":1559},
#     {"x":1202,"y":1561},{"x":1203,"y":1561},{"x":1201,"y":1560},{"x":1201,"y":1561},
#     {"x":1201,"y":1559},{"x":1200,"y":1559},{"x":1200,"y":1560}]
# # KIV: The following 4 variables would be use for later calculations.
# minXVal=None
# maxXVal=None

# minYVal=None
# maxYVal=None

# for item in xy:
#   x=item["x"]
#   y=item["y"]
#   if (minXVal is None) or (x <= minXVal):
#       minXVal=x

#   if (minYVal is None) or (y <= minYVal):
#       minYVal=y

#   if (maxXVal is None) or (x >= maxXVal):
#       maxXVal=x

#   if (maxYVal is None) or (y >= maxYVal):
#       maxYVal=y

#   feature={
#     "type":"Feature",
#     "geometry": {
#       "type": "Point",
#       "coordinates": [
#         num2deg(x, y, zoom)[1],
#         num2deg(x, y, zoom)[0]
#       ]
#      },
#       "properties": {
#         "zoom": zoom
#       } 
#   }
#   geojsonObj["features"].append(feature)
    
# geojson=str(geojsonObj).replace("'","\"")

# # Proceed to output the GeoJSON into a file named: output.geojson
# geojson_file = open("output.geojson", "w")
# geojson_file.write(geojson)
# geojson_file.close()

# print("GeoJSON has been saved to file: output.geojson")


# minZoomLevel=11 # IMPORTANT! Ensure that your minZoomLevel is the same as the one you decided intially
# maxZoomLevel=15 # The upper limit of zoom level set on the basemap

# directoryPrefix="terrain/" # assume it is being saved in a folder named toner

# basemapUrlPrefix="http://tile.stamen.com/terrain/"
# basemapUrlSuffix=".png"

# def initDirectoryStructure(n): # where n refers to the zoom level
#     minX=(minXVal/4)*4*pow(2,n-minZoomLevel)
#     maxX=(maxXVal/4)*4*pow(2,n-minZoomLevel)
    
#     minY=(minYVal/6)*6*pow(2,n-minZoomLevel)
#     maxY=(maxYVal/6)*6*pow(2,n-minZoomLevel)
    
#     minX=int(minX)
#     maxX=int(maxX)
#     minY=int(minY)
#     maxY=int(maxY)
    
#     for x in range(minX,maxX+1,1):
#       for y in range(minY,maxY+1,1):
#             directory=directoryPrefix+str(n) + "/" + str(x)
#             if not os.path.exists(directory):
#                 os.makedirs(directory)

# # Execute function to create folder structures for zoom levels minZoomLevel to maxZoomLevel (inclusive)
# for z in range(minZoomLevel,maxZoomLevel+1,1):
#     initDirectoryStructure(z)
    
# print("Folder structure for zoom levels " + str(minZoomLevel) + " to " + str(maxZoomLevel) + " has been created.")



# def streamTileImages(n): # where n refers to the zoom level
#     minX=(minXVal/4)*4*pow(2,n-minZoomLevel)
#     maxX=(maxXVal/4)*4*pow(2,n-minZoomLevel)
    
#     minY=(minYVal/6)*6*pow(2,n-minZoomLevel)
#     maxY=(maxYVal/6)*6*pow(2,n-minZoomLevel)
    
#     minX=int(minX)
#     maxX=int(maxX)
#     minY=int(minY)
#     maxY=int(maxY)
    
    # for x in range(minX,maxX+1,1):
    #     for y in range(minY,maxY+1,1):
            
    #         # proceed to save it as a local image file in the folders created in step 3
    #         save_as_filename=directoryPrefix+str(n) + "/" + str(x) + "/" + str(y) + ".png"
    #         if (not os.path.isfile(save_as_filename) or
    #                 os.path.getsize(save_as_filename) < 1000):
    #             # send a HTTP request to retrieve the tile image file
    #             basemapTileUrl=basemapUrlPrefix+str(n) + "/" + str(x) + "/" + str(y) + basemapUrlSuffix
    #             r = requests.get(basemapTileUrl)
    #             with open(save_as_filename,"wb") as local_tile_image:
    #                 local_tile_image.write(r.content) 
    #                 print('.', end='')
    #     print(F"Zoom {n}: {x-minX}/{maxX+1-minX} done")

# for z in range(minZoomLevel,maxZoomLevel+1,1):
#     streamTileImages(z)
    
# print("All map tile images for zoom levels " + str(minZoomLevel) + " to " + str(maxZoomLevel) + " have been saved to local directories.")


# from flask import Flask, render_template
# app=Flask(__name__)
# @app.route('/')
# def root():
#    markers=[
#    {
#    'lat':0,
#    'lon':0,
#    'popup':'This is the middle of the map.'
#     }
#    ]
#    return render_template('map.html',markers=markers )
# if __name__ == '__main__':
#    app.run(host="localhost", port=8080, debug=True)