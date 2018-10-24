import lanelet2
tr = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.Car)
map = lanelet2.io.load("../../lanelet2_maps/res/mapping_example.osm")
rg = lanelet2.routing.RoutingGraph(map, tr)


idCount = 1
def makePolygon(llt, ofType="route"):
    global idCount
    idCount += 1
    poly = lanelet2.core.Polygon3d(idCount, [p for p in llt.leftBound] + [p for p in llt.rightBound.invert()] + [llt.leftBound[0]], llt.attributes)
    poly.attributes["type"] = ofType
    poly.attributes["area"] = "yes"
    return poly


def makeRoute(fromId, toId):
    ll43694 = map.laneletLayer.get(fromId)
    ll45260 = map.laneletLayer.get(toId)
    p=rg.shortestPath(ll43694, ll45260)
    corr = rg.getCorridor(ll43694, ll45260)
    llts = [map.laneletLayer.get(llt.id) for llt in p.lanelets()]
    confLlts = [map.laneletLayer.get(llt.id) for llt in corr.allConflictingInMap() if llt.invert() not in p.lanelets()]
    polys = [makePolygon(llt) for llt in llts]
    confPolys = [makePolygon(llt, "conflicting") for llt in confLlts]
    return polys + confPolys
    
routePolygons = makeRoute(43694,45260)
route = lanelet2.core.createMapFromPolygons(routePolygons)
