export OSM_DIR=/data/osm
export DB_DIR=${OSM_DIR}/db
export GZ_MAP_FILE_AREA=north-america
export GZ_MAP_FILE_NAME=us-latest.osm.bz2
export GZ_MAP_FILE=${OSM_DIR}/${GZ_MAP_FILE_NAME}

# Remove legacy compressed map file if existing
[ -f $GZ_MAP_FILE ] && rm -rf $GZ_MAP_FILE

# WD
cd $OSM_DIR

# Download map file
wget https://download.geofabrik.de/${GZ_MAP_FILE_AREA}/${GZ_MAP_FILE_NAME}

if [[ "$?" != 0 ]]; then
  echo "Error downloading map file"
else
  echo "Successfuly downloaded map file"
  # Remove current db dir if exisiting
  [ -f $DB_DIR ] && rm -rf $DB_DIR
  # Populate data-base (U can do this outside and after ftp put in DB_DIR)
  /data/src/bin/init_osm3s.sh $GZ_MAP_FILE_NAME $DB_DIR $EXEC_DIR --meta 
  # Remove compressed map files after expanding
  rm -rf $GZ_MAP_FILE_NAME
fi
