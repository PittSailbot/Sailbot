var map = L.map('map').fitWorld();
L.tileLayer('http://localhost:3650/api/tiles/osm-2020-02-10-v3/{z}/{x}/{y}', {
attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
}).addTo(map);