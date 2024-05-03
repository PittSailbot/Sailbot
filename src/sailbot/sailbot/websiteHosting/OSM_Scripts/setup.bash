
docker volume create osm-data
docker run --name osm_container  -v /home/sailbot/OSM/us-northeast-latest.osm.pbf:/data/region.osm.pbf  -v osm-data:/data/database/  overv/openstreetmap-tile-server  import
echo "If this did not work properly try removing the docker container associated with it and run docker rm osm-data"
