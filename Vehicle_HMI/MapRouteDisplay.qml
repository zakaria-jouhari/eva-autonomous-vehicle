// MapRouteDisplay.qml
// Composant pour afficher la route ROS2 sur la carte (Qt 6 compatible)

import QtQuick 2.15
import QtLocation 6.2
import QtPositioning 6.2

Item {
    id: root
    
    // Propriétés exposées
    property var map: null
    property var routeService: null
    
    // Origine (Casablanca par défaut)
    property double originLat: 33.5731
    property double originLon: -7.5898
    
    // Fonctions de conversion GPS <-> Local
    function localToGps(x, y) {
        var metersPerDegreeLat = 111320.0;
        var metersPerDegreeLon = 111320.0 * Math.cos(originLat * Math.PI / 180.0);
        
        var lat = originLat + (y / metersPerDegreeLat);
        var lon = originLon + (x / metersPerDegreeLon);
        
        return {lat: lat, lon: lon};
    }
    
    function gpsToLocal(lat, lon) {
        var metersPerDegreeLat = 111320.0;
        var metersPerDegreeLon = 111320.0 * Math.cos(originLat * Math.PI / 180.0);
        
        var x = (lon - originLon) * metersPerDegreeLon;
        var y = (lat - originLat) * metersPerDegreeLat;
        
        return {x: x, y: y};
    }
    
    // Route globale (OSRM)
    MapPolyline {
        id: globalRouteLine
        line.width: 6
        line.color: "#4285F4"  // Bleu Google Maps
        visible: routeService && routeService.globalPath && routeService.globalPath.length > 0
        
        path: {
            if (!routeService || !routeService.globalPath) return [];
            
            var coords = [];
            for (var i = 0; i < routeService.globalPath.length; i++) {
                var pt = routeService.globalPath[i];
                var gps = localToGps(pt.x, pt.y);
                coords.push(QtPositioning.coordinate(gps.lat, gps.lon));
            }
            return coords;
        }
    }
    
    // Route locale (spline lissée)
    MapPolyline {
        id: localRouteLine
        line.width: 4
        line.color: "#34A853"  // Vert Google Maps
        visible: routeService && routeService.localPath && routeService.localPath.length > 0
        
        path: {
            if (!routeService || !routeService.localPath) return [];
            
            var coords = [];
            for (var i = 0; i < routeService.localPath.length; i++) {
                var pt = routeService.localPath[i];
                var gps = localToGps(pt.x, pt.y);
                coords.push(QtPositioning.coordinate(gps.lat, gps.lon));
            }
            return coords;
        }
    }
    
    // Marqueur de départ
    MapQuickItem {
        id: startMarker
        visible: routeService && routeService.globalPath && routeService.globalPath.length > 0
        
        coordinate: {
            if (!routeService || !routeService.globalPath || routeService.globalPath.length === 0) {
                return QtPositioning.coordinate(0, 0);
            }
            var pt = routeService.globalPath[0];
            var gps = localToGps(pt.x, pt.y);
            return QtPositioning.coordinate(gps.lat, gps.lon);
        }
        
        anchorPoint.x: startIcon.width / 2
        anchorPoint.y: startIcon.height
        
        sourceItem: Rectangle {
            id: startIcon
            width: 30
            height: 30
            radius: 15
            color: "#4285F4"
            border.color: "white"
            border.width: 3
            
            Text {
                anchors.centerIn: parent
                text: "A"
                color: "white"
                font.bold: true
                font.pixelSize: 16
            }
        }
    }
    
    // Marqueur d'arrivée
    MapQuickItem {
        id: endMarker
        visible: routeService && routeService.globalPath && routeService.globalPath.length > 0
        
        coordinate: {
            if (!routeService || !routeService.globalPath || routeService.globalPath.length === 0) {
                return QtPositioning.coordinate(0, 0);
            }
            var pt = routeService.globalPath[routeService.globalPath.length - 1];
            var gps = localToGps(pt.x, pt.y);
            return QtPositioning.coordinate(gps.lat, gps.lon);
        }
        
        anchorPoint.x: endIcon.width / 2
        anchorPoint.y: endIcon.height
        
        sourceItem: Rectangle {
            id: endIcon
            width: 30
            height: 30
            radius: 15
            color: "#EA4335"  // Rouge Google Maps
            border.color: "white"
            border.width: 3
            
            Text {
                anchors.centerIn: parent
                text: "B"
                color: "white"
                font.bold: true
                font.pixelSize: 16
            }
        }
    }
    
    // Position actuelle du véhicule
    MapQuickItem {
        id: vehicleMarker
        coordinate: QtPositioning.coordinate(originLat, originLon)
        visible: true
        
        anchorPoint.x: vehicleIcon.width / 2
        anchorPoint.y: vehicleIcon.height / 2
        
        sourceItem: Rectangle {
            id: vehicleIcon
            width: 40
            height: 40
            radius: 20
            color: "#34A853"
            border.color: "white"
            border.width: 4
            
            Rectangle {
                anchors.centerIn: parent
                width: 8
                height: 8
                radius: 4
                color: "white"
            }
        }
    }
    
    // Animation de zoom sur la route
    function zoomToRoute() {
        if (!routeService || !routeService.globalPath || routeService.globalPath.length === 0) {
            return;
        }
        
        // Calculer les limites
        var minLat = 999, maxLat = -999;
        var minLon = 999, maxLon = -999;
        
        for (var i = 0; i < routeService.globalPath.length; i++) {
            var pt = routeService.globalPath[i];
            var gps = localToGps(pt.x, pt.y);
            
            minLat = Math.min(minLat, gps.lat);
            maxLat = Math.max(maxLat, gps.lat);
            minLon = Math.min(minLon, gps.lon);
            maxLon = Math.max(maxLon, gps.lon);
        }
        
        // Centre de la vue
        var centerLat = (minLat + maxLat) / 2;
        var centerLon = (minLon + maxLon) / 2;
        
        if (map) {
            map.center = QtPositioning.coordinate(centerLat, centerLon);
            
            // Ajuster le zoom pour voir toute la route
            var latDelta = maxLat - minLat;
            var lonDelta = maxLon - minLon;
            var maxDelta = Math.max(latDelta, lonDelta);
            
            // Calcul approximatif du niveau de zoom
            if (maxDelta > 0.5) map.zoomLevel = 10;
            else if (maxDelta > 0.1) map.zoomLevel = 12;
            else if (maxDelta > 0.05) map.zoomLevel = 13;
            else if (maxDelta > 0.01) map.zoomLevel = 15;
            else map.zoomLevel = 16;
        }
    }
    
    // Connexion aux changements de route
    Connections {
        target: routeService
        function onRouteReady() {
            console.log("QML: Route ready, zooming to fit");
            zoomToRoute();
        }
    }
}
