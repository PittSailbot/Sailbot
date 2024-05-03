docker network create osm_network
docker run --name osm_server --network osm_network -p 8080:80 -v osm-data:/data/database -d overv/openstreetmap-tile-server run
docker run -d --name nginx_proxy --network osm_network -p 80:80 -p 443:443 nginx:latest
docker cp /home/sailbot/OSM/nginx_conf nginx_proxy:/etc/nginx/nginx.conf
docker cp /home/sailbot/sailbot/cert.pem nginx_proxy:/home/cert.pem
docker cp /home/sailbot/sailbot/key.pem nginx_proxy:/home/key.pem
docker restart nginx_proxy 
