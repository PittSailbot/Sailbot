mkdir ./volumeBackup/tmp
tar xzf ./volumeBackup/osm_backup.tar.gz -C ./volumeBackup/tmp
docker volume create restored-osm-data
docker run --rm -v ./volumeBackup/tmp:/source -v restored-osm-data:/destination busybox cp -r /source/. /destination/
rm ./volumeBackup/tmp -r
