docker run --rm -v osm-data:/data -v /home/sailbot/OSM/volumeBackup:/backup busybox tar czf /backup/osm_backup.tar.gz -C /data .
